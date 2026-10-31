#include "zr10_ros2/media.h"
#include <curl/curl.h>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <algorithm>
using json = nlohmann::json;
namespace fs = std::filesystem;
namespace {
struct Transfer { std::string body; std::ofstream* file=nullptr; const std::atomic<bool>* stop; size_t bytes=0; };
size_t writeData(char* p, size_t size, size_t count, void* opaque) {
    auto& t=*static_cast<Transfer*>(opaque); size_t n=size*count;
    if (t.bytes+n > (t.file ? 128u*1024u*1024u : 8u*1024u*1024u)) return 0;
    try {
        if (t.file) { t.file->write(p,n); if (!*t.file) return 0; }
        else t.body.append(p,n);
        t.bytes+=n; return n;
    } catch (...) { return 0; }
}
int progress(void* p, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
    return static_cast<const std::atomic<bool>*>(p)->load() ? 1 : 0;
}
void get(const std::string& url, Transfer& t, long timeout) {
    std::unique_ptr<CURL,decltype(&curl_easy_cleanup)> c(curl_easy_init(),curl_easy_cleanup);
    if (!c) throw std::runtime_error("curl initialization failed");
    curl_easy_setopt(c.get(),CURLOPT_URL,url.c_str());
    curl_easy_setopt(c.get(),CURLOPT_PROTOCOLS,CURLPROTO_HTTP|CURLPROTO_HTTPS);
    curl_easy_setopt(c.get(),CURLOPT_FOLLOWLOCATION,0L);
    curl_easy_setopt(c.get(),CURLOPT_NOSIGNAL,1L);
    curl_easy_setopt(c.get(),CURLOPT_CONNECTTIMEOUT,3L);
    curl_easy_setopt(c.get(),CURLOPT_TIMEOUT,timeout);
    curl_easy_setopt(c.get(),CURLOPT_WRITEFUNCTION,writeData);
    curl_easy_setopt(c.get(),CURLOPT_WRITEDATA,&t);
    curl_easy_setopt(c.get(),CURLOPT_NOPROGRESS,0L);
    curl_easy_setopt(c.get(),CURLOPT_XFERINFOFUNCTION,progress);
    curl_easy_setopt(c.get(),CURLOPT_XFERINFODATA,t.stop);
    auto rc=curl_easy_perform(c.get()); long status=0;
    curl_easy_getinfo(c.get(),CURLINFO_RESPONSE_CODE,&status);
    if (rc!=CURLE_OK || status!=200) throw std::runtime_error("HTTP " + std::to_string(status) + ": " + curl_easy_strerror(rc));
}
json query(const MediaOptions& o,const std::string& path,const std::atomic<bool>& stop) {
    Transfer t{"",nullptr,&stop}; get(o.base+path,t,o.timeout);
    auto j=json::parse(t.body);
    if (!j.at("success").get<bool>() || j.at("code").get<int>()!=200)
        throw std::runtime_error("Camera media API rejected " + path +
            " (code " + j.at("code").dump() + "): " + j.value("message", std::string("No message")));
    return j.at("data");
}
std::string encodeQuery(const std::string& value) {
    // Encode UTF-8 bytes and reserved query characters; paths come from the camera.
    static const char* hex="0123456789ABCDEF";
    std::string encoded;
    for (unsigned char c:value) {
        if ((c>='a' && c<='z') || (c>='A' && c<='Z') ||
            (c>='0' && c<='9') || c=='-' || c=='_' || c=='.' || c=='~') encoded+=c;
        else { encoded+='%'; encoded+=hex[c>>4]; encoded+=hex[c&15]; }
    }
    return encoded;
}
std::string filename(const std::string& url) {
    // Collision-free, path-safe encoding of the full remote URL, split into directories
    // to stay below NAME_MAX. The stable URL acts as the persistent identity.
    static const char* hex="0123456789abcdef"; std::string result;
    for (unsigned char c:url) { result+=hex[c>>4]; result+=hex[c&15]; }
    if (result.empty() || result.size()>3000) throw std::runtime_error("Invalid media URL length");
    for (size_t i=120;i<result.size();i+=121) result.insert(i,"/");
    return result+".jpg";
}
bool jpegComplete(const fs::path& path) {
    // Observed ZR10 firmware pads JPEGs with zeros after FF D9.
    // Check the last nonzero byte, not an arbitrary embedded/thumbnail marker.
    // This is a framing check, not a full JPEG decode. Preserve the original bytes.
    std::ifstream f(path,std::ios::binary);
    unsigned char start[2]{};
    if (!f.read(reinterpret_cast<char*>(start),2) || start[0]!=0xff || start[1]!=0xd8) return false;
    f.seekg(0,std::ios::end);
    const auto end=f.tellg();
    if (end==std::streampos(-1)) return false;
    std::streamoff position=static_cast<std::streamoff>(end);
    if (position<4) return false;
    char buffer[4096];
    while (position>2) {
        const auto count=std::min<std::streamoff>(position-2,sizeof(buffer));
        const auto offset=position-count;
        f.seekg(offset);
        if (!f.read(buffer,static_cast<std::streamsize>(count))) return false;
        for (std::streamoff i=count;i>0;--i) {
            const auto index=i-1;
            const auto byte=static_cast<unsigned char>(buffer[index]);
            if (byte==0) continue;
            if (byte!=0xd9 || offset+index<3) return false;
            unsigned char previous=0;
            f.seekg(offset+index-1);
            return bool(f.read(reinterpret_cast<char*>(&previous),1)) && previous==0xff;
        }
        position=offset;
    }
    return false;
}
}
void syncPhotos(const MediaOptions& o,const std::atomic<bool>& stop) {
    while (!stop) {
        auto next=std::chrono::steady_clock::now()+std::chrono::duration<double>(o.interval);
        try {
            fs::create_directories(o.directory);
            auto inventory=query(o,"/api/v1/getdirectories?media_type=0",stop);
            const auto& directories=inventory.at("directories");
            if (!directories.is_array()) throw std::runtime_error("Invalid directory list");
            for (const auto& directory:directories) {
                if (stop) break;
                try {
                    const auto remotePath=directory.at("path").get<std::string>();
                    if (remotePath.empty()) throw std::runtime_error("Empty directory path returned by camera");
                    const auto parameters="?media_type=0&path="+encodeQuery(remotePath);
                    int total=query(o,"/api/v1/getmediacount"+parameters,stop).at("count").get<int>();
                    if (total<0 || total>1000000) throw std::runtime_error("Invalid photo count");
                    for (int start=0;start<total && !stop;start+=100) {
                        auto data=query(o,"/api/v1/getmedialist"+parameters+"&start="+std::to_string(start)+"&count=100",stop);
                        const auto& entries=data.at("list");
                        if (!entries.is_array()) throw std::runtime_error("Invalid media list");
                        for (const auto& entry:entries) {
                            if (stop) break;
                            try {
                                const auto url=entry.at("url").get<std::string>();
                                const auto name=entry.at("name").get<std::string>();
                                fs::path target=fs::path(o.directory)/filename(url);
                                if (fs::exists(target)) continue;
                                fs::create_directories(target.parent_path()); auto part=target.string()+".part";
                                try {
                                    std::ofstream out(part,std::ios::binary|std::ios::trunc);
                                    if (!out) throw std::runtime_error("Cannot open download file");
                                    Transfer t{"",&out,&stop}; get(url,t,o.timeout); out.flush();
                                    if (!out) throw std::runtime_error("Cannot flush download");
                                    out.close();
                                    if (!jpegComplete(part)) throw std::runtime_error("Incomplete/non-JPEG photo; will retry");
                                    fs::rename(part,target);
                                    std::cerr << "Downloaded " << name << " -> " << target << '\n';
                                } catch (...) { fs::remove(part); throw; }
                            } catch (const std::exception& e) { if (!stop) std::cerr << "Photo download: " << e.what() << '\n'; }
                        }
                    }
                } catch (const std::exception& e) {
                    if (!stop) std::cerr << "Media directory: " << e.what() << '\n';
                }
            }
        } catch (const std::exception& e) { if (!stop) std::cerr << "Media sync: " << e.what() << '\n'; }
        // No overlapping scans or catch-up burst when HTTP takes longer than interval.
        while (!stop && std::chrono::steady_clock::now()<next) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
