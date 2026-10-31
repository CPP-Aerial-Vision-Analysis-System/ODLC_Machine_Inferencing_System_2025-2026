#pragma once
#include <atomic>
#include <string>
struct MediaOptions {
    std::string base, directory = "photos";
    double interval = 2;
    long timeout = 15;
};
// Blocking worker; run in its own thread. CURL must be globally initialized first.
void syncPhotos(const MediaOptions& options, const std::atomic<bool>& stop);
