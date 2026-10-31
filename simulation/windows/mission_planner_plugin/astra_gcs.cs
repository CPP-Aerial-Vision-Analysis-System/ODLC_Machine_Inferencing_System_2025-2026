/*
 * simulation/windows/mission_planner_plugin/astra_gcs.cs -- PURPOSE: make Mission Planner the ONE ground-station
 * screen of the ASTRA search mission. It replaces the old "ASTRA Simulation" demo panel. Mission Planner compiles this
 * file by itself when it starts (it is a Mission Planner plugin: a C# file in Mission Planner's "plugins" folder; the
 * installer simulation/windows/install_mp_plugin.ps1, or mission_planner.sh start, puts it there).
 *
 * What it adds to Mission Planner:
 *   1. THE DRONE CAMERA IN THE HUD, with no clicks. When the camera stream answers (the simulator serves it at
 *      http://127.0.0.1:8081/pov.mjpg, see sim_demo/pov_server.py), the HUD background becomes the live camera picture;
 *      when the stream goes away the normal artificial horizon comes back. HUD right-click > "ASTRA camera" turns it
 *      on or off and changes the address.
 *   2. AN "ASTRA" TAB on the DATA screen (it opens by default), the job the old panel did:
 *        - the mission phase strip: Waiting > Take-off > Search > Process > Targets > Drop > Return > Landed,
 *        - one plain sentence about what the drone is doing right now,
 *        - big readouts: flight mode, height, speed, battery, GPS, distance home, time in the air, waypoint,
 *        - one card each for the HIKER and the TENT: detections, the target position chosen, the payload drop,
 *        - an event log (the autopilot's and the team code's messages), warnings in amber,
 *        - buttons: ARM + START MISSION and RETURN HOME (RTL), each with a confirmation box; in the simulator also
 *          TAKE PHOTO, CAMERA DOWN, CAMERA FORWARD and START OVER (run through WSL by simulation/scripts/mp_actions.sh).
 *   3. MAP MARKERS for the chosen targets (from the team's "Aim ..." messages) and for the drop points.
 *
 * Where the information comes from: ONLY what Mission Planner already receives over MAVLink, so it works the same with
 * the real drone over the telemetry radio. The team's flight code (main_controller.py) sends status texts such as
 * "Detected tent", "All 97 images processed. Selecting targets.", "Aim tent 36.2149350,-96.0069430",
 * "Targets: tent@11 -> person@12. Diverting.", "Over tent: loiter 6s then drop" and "Servo 13 -> 1400"; the optional
 * Jetson reporter (lattice_bridge) adds "TGT p 36.214935 -96.006203 95 12"; the autopilot supplies the flight mode,
 * arming, height, speed, battery and GPS. Only the camera picture and the simulator buttons use something else (the
 * camera stream, and WSL).
 *
 * Safety: it never changes the flight by itself. The two flight buttons ask "are you sure" first and then do exactly
 * what Mission Planner's own buttons do (set GUIDED, arm, set AUTO / set RTL). Every part catches its own errors: a
 * problem here shows up as a line in the event log and in the status file, never as a crash of Mission Planner.
 *
 * Observability: every 2 s it writes Documents\Mission Planner\astra_plugin_status.json (phase, camera pictures, what
 * it has seen), so scripts and teammates can check it works without looking at the screen.
 *
 * Settings (Mission Planner's config.xml, changed through the HUD menu or by hand while Mission Planner is closed):
 *   astra_camera_on       "true"/"false"   show the camera in the HUD (default true)
 *   astra_camera_url      MJPEG address    default http://127.0.0.1:8081/pov.mjpg
 *   astra_sim_buttons     auto|on|off      simulator buttons: auto = only when connected to the simulator (SITL)
 *   astra_wsl_distro      WSL distribution for the simulator buttons (default Ubuntu-22.04)
 *   astra_wsl_repo        the repository copy inside WSL (default ~/ODLC_Machine_Inferencing_System_2025-2026)
 *   astra_drop_channels   servo channel -> payload, default "11:person,13:tent";  astra_drop_open_pwm default 1400
 *
 * Two technical notes for whoever changes this file:
 *   - Mission Planner ships its OWN copy of most System.Drawing types (MissionPlanner.Drawing, built on SkiaSharp),
 *     next to .NET's. Naming Font, Pen, Brush, Bitmap or Image in this file would therefore be ambiguous. So it never
 *     names them: fonts are made by reflection (Ui.SetFont), the phase strip is plain labels, and camera pictures are
 *     decoded with .NET's Image.FromStream found through Form.BackgroundImage (the type the HUD's bgimage expects).
 *     Color is safe (it exists only once).
 *   - Plain C# 5, so both Mission Planner's compiler and the old .NET Framework compiler accept it. Check it with
 *     simulation/windows/check_mp_plugin.ps1 before installing.
 * Written for Mission Planner 1.3.83 (FlightData.myhud, CurrentState, MAVLinkInterface, PluginHost).
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using MissionPlanner;
using MissionPlanner.GCSViews;
using MissionPlanner.Utilities;

namespace AstraGcs
{
    /// <summary>The mission phases, in flight order (the strip shows them left to right).</summary>
    public enum Phase { Waiting, Takeoff, Search, Process, Targets, Drop, Return, Landed }

    /// <summary>What is known about one target kind (the hiker = "person", or the tent).</summary>
    public class TargetInfo
    {
        public string Kind;
        public int Detections;
        public DateTime FirstDetected = DateTime.MinValue;
        public int BestReport = -1;              // best confidence (percent) of a TGT radio report, -1 = none
        public double ReportLat, ReportLon;
        public bool HasAim;
        public double AimLat, AimLon;
        public bool Dropping;
        public bool Dropped;
        public DateTime DropAt = DateTime.MinValue;
        public double DropLat, DropLon;

        /// <summary>"hiker" or "tent", for sentences.</summary>
        public string Word { get { return Kind == "person" ? "hiker" : "tent"; } }
    }

    /// <summary>One line of the event log.</summary>
    public class EventItem
    {
        public DateTime Time;
        public string Source;
        public string Text;
        public int Level;            // 0 normal, 1 good news, 2 warning
    }

    /// <summary>The colours of the tab (dark, like Mission Planner's own theme).</summary>
    public static class Theme
    {
        public static readonly Color Background = Color.FromArgb(30, 33, 38);
        public static readonly Color Card = Color.FromArgb(43, 47, 54);
        public static readonly Color Line = Color.FromArgb(70, 76, 86);
        public static readonly Color Text = Color.FromArgb(232, 236, 241);
        public static readonly Color Muted = Color.FromArgb(150, 160, 172);
        public static readonly Color Accent = Color.FromArgb(47, 129, 247);
        public static readonly Color Good = Color.FromArgb(46, 160, 67);
        public static readonly Color Warn = Color.FromArgb(210, 153, 34);
        public static readonly Color Bad = Color.FromArgb(248, 81, 73);
        public static readonly Color DoneFill = Color.FromArgb(32, 64, 44);
        public static readonly Color DoneText = Color.FromArgb(160, 220, 170);
    }

    /// <summary>Small helpers that avoid naming the drawing types Mission Planner duplicates (see the file header).</summary>
    public static class Ui
    {
        private static readonly PropertyInfo FontProperty = typeof(Control).GetProperty("Font");
        private static readonly Type FontType = FontProperty.PropertyType;
        private static readonly Type FontStyleType = FontType.Assembly.GetType("System.Drawing.FontStyle");
        private static readonly Dictionary<string, object> Fonts = new Dictionary<string, object>();

        /// <summary>Give `control` the Segoe UI font at `size` points, bold or regular (fonts are shared and cached).</summary>
        public static void SetFont(Control control, float size, bool bold)
        {
            try
            {
                string key = size.ToString(CultureInfo.InvariantCulture) + (bold ? "b" : "r");
                object font;
                lock (Fonts)
                {
                    if (!Fonts.TryGetValue(key, out font))
                    {
                        object style = Enum.ToObject(FontStyleType, bold ? 1 : 0);
                        font = Activator.CreateInstance(FontType, new object[] { "Segoe UI", size, style });
                        Fonts[key] = font;
                    }
                }
                FontProperty.SetValue(control, font, null);
            }
            catch (Exception)
            {
                // keep the default font
            }
        }

        /// <summary>Centre a label's text (ContentAlignment.MiddleCenter = 32; the type itself is duplicated, see the header).</summary>
        public static void Center(Label label)
        {
            try
            {
                var property = typeof(Label).GetProperty("TextAlign");
                property.SetValue(label, Enum.ToObject(property.PropertyType, 32), null);
            }
            catch (Exception)
            {
                // left-aligned text is fine too
            }
        }

        /// <summary>A label with text, colours and a font, ready to be docked.</summary>
        public static Label Label(string text, float size, bool bold, Color color)
        {
            var label = new Label { Text = text, ForeColor = color, BackColor = Color.Transparent, AutoEllipsis = true };
            SetFont(label, size, bold);
            return label;
        }
    }

    /// <summary>The mission phase strip: one box per phase, done ones ticked, the current one filled.</summary>
    public class PhaseStrip : TableLayoutPanel
    {
        public static readonly string[] Names = { "Waiting", "Take-off", "Search", "Process", "Targets", "Drop", "Return", "Landed" };
        private readonly Label[] boxes = new Label[Names.Length];
        private int current = -1;

        public PhaseStrip()
        {
            ColumnCount = Names.Length;
            RowCount = 1;
            BackColor = Theme.Background;
            for (int i = 0; i < Names.Length; i++)
            {
                ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / Names.Length));
                var box = Ui.Label(Names[i], 9f, true, Theme.Muted);
                box.Dock = DockStyle.Fill;
                Ui.Center(box);
                box.BackColor = Theme.Card;
                box.Margin = new Padding(i == 0 ? 0 : 2, 0, 0, 0);
                boxes[i] = box;
                Controls.Add(box, i, 0);
            }
            Current = 0;
        }

        /// <summary>The index of the current phase (0 = Waiting).</summary>
        public int Current
        {
            get { return current; }
            set
            {
                if (value == current) return;
                current = value;
                for (int i = 0; i < boxes.Length; i++)
                {
                    boxes[i].Text = (i < current ? "\u2713 " : "") + Names[i];
                    boxes[i].BackColor = i < current ? Theme.DoneFill : (i == current ? Theme.Accent : Theme.Card);
                    boxes[i].ForeColor = i == current ? Color.White : (i < current ? Theme.DoneText : Theme.Muted);
                }
            }
        }
    }

    public class AstraPlugin : MissionPlanner.Plugin.Plugin
    {
        private const string PluginVersion = "1.0";
        private const string DefaultCameraUrl = "http://127.0.0.1:8081/pov.mjpg";

        // ---------------------------------------------------------------- mission state (plugin thread, under `sync`)
        private readonly object sync = new object();
        private Phase phase = Phase.Waiting;
        private bool flightStarted;
        private bool wasArmed;
        private bool searchDone;
        private bool targetsChosen;
        private bool returning;
        private int photosProcessed = -1;
        private readonly Dictionary<string, TargetInfo> targets = new Dictionary<string, TargetInfo>();
        private readonly List<EventItem> events = new List<EventItem>();
        private readonly List<EventItem> pendingEvents = new List<EventItem>();
        private readonly Dictionary<string, DateTime> lastMessageTime = new Dictionary<string, DateTime>();
        private readonly HashSet<string> seenMessages = new HashSet<string>();
        private readonly Queue<string> seenOrder = new Queue<string>();
        private DateTime lastMessageAt = DateTime.MinValue;
        private int messagesSeen;
        private int companionTexts;                    // STATUSTEXT packets from the Jetson side (any component but the autopilot)
        private int statustextSubscription = -1;
        private readonly Queue<EventItem> rawTexts = new Queue<EventItem>();   // filled by Mission Planner's reader thread
        private string lastError = "";
        private DateTime lastStatusWrite = DateTime.MinValue;
        private bool simulator;
        private bool mapDirty;

        // ---------------------------------------------------------------- camera (its own thread)
        private static readonly Type ImageType = typeof(Form).GetProperty("BackgroundImage").PropertyType;   // .NET's Image
        private static readonly MethodInfo ImageFromStream = ImageType.GetMethod("FromStream", new[] { typeof(Stream) });
        private Thread cameraThread;
        private volatile bool stopping;
        private volatile bool capturing;
        private long frames;
        private DateTime lastFrameAt = DateTime.MinValue;
        private string cameraState = "starting";
        private readonly Queue<IDisposable> shownPictures = new Queue<IDisposable>();

        // ---------------------------------------------------------------- UI (UI thread only)
        private TabPage page;
        private PhaseStrip strip;
        private Label statusLine;
        private Label linkLine;
        private readonly Dictionary<string, Label> tiles = new Dictionary<string, Label>();
        private readonly Dictionary<string, Label[]> cards = new Dictionary<string, Label[]>();
        private readonly Dictionary<string, Panel> cardEdges = new Dictionary<string, Panel>();
        private ListView log;
        private FlowLayoutPanel simButtons;
        private GMapOverlay overlay;

        public override string Name { get { return "ASTRA ground station"; } }
        public override string Version { get { return PluginVersion; } }
        public override string Author { get { return "ASTRA team"; } }

        public override bool Init()
        {
            loopratehz = 2;
            targets["person"] = new TargetInfo { Kind = "person" };
            targets["tent"] = new TargetInfo { Kind = "tent" };
            return true;
        }

        public override bool Loaded()
        {
            try
            {
                BuildTab();
            }
            catch (Exception ex)
            {
                lastError = "building the ASTRA tab: " + ex.Message;
            }
            try { AddHudMenu(); } catch (Exception ex) { lastError = "HUD menu: " + ex.Message; }
            try { AddMapOverlay(); } catch (Exception ex) { lastError = "map: " + ex.Message; }
            try
            {
                // Every STATUSTEXT packet, from any system and component (0, 0 = all). Mission Planner files the texts of
                // other components (the Jetson's MAVROS is 1:191) under that component, not under the autopilot, so
                // listening to the packets themselves is the reliable way to read the team code's messages.
                statustextSubscription = Host.comPort.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.STATUSTEXT, OnStatusText, 0, 0, false);
            }
            catch (Exception ex)
            {
                lastError = "status text subscription: " + ex.Message;
            }
            AddEvent("ASTRA", "ASTRA ground station plugin " + PluginVersion + " loaded", 1);
            stopping = false;
            cameraThread = new Thread(CameraLoop) { IsBackground = true, Name = "astra-camera" };
            cameraThread.Start();
            WriteStatus();
            return true;
        }

        public override bool Loop()
        {
            try
            {
                Tick();
            }
            catch (Exception ex)
            {
                lastError = "update: " + ex.Message;
            }
            return true;
        }

        public override bool Exit()
        {
            stopping = true;
            try { if (statustextSubscription >= 0) Host.comPort.UnSubscribeToPacketType(statustextSubscription); } catch (Exception) { }
            ClearHud();
            return true;
        }

        /// <summary>Mission Planner's reader thread: keep a copy of every non-autopilot status text (fast; parsed in Tick).</summary>
        private bool OnStatusText(MAVLink.MAVLinkMessage message)
        {
            try
            {
                if (message.compid == 1) return true;          // the autopilot's own texts arrive through CurrentState.messages
                var packet = (MAVLink.mavlink_statustext_t)message.data;
                string text = Encoding.ASCII.GetString(packet.text).TrimEnd('\0').Trim();
                if (text.Length == 0) return true;
                lock (rawTexts)
                {
                    rawTexts.Enqueue(new EventItem { Time = DateTime.Now, Source = message.sysid + ":" + message.compid, Text = text });
                    while (rawTexts.Count > 500) rawTexts.Dequeue();
                }
            }
            catch (Exception)
            {
                // a malformed packet: ignore it
            }
            return true;
        }

        // =============================================================== settings
        private static string Setting(string key, string fallback)
        {
            try
            {
                string value = Settings.Instance[key];
                return string.IsNullOrEmpty(value) ? fallback : value;
            }
            catch (Exception)
            {
                return fallback;
            }
        }

        private static void SaveSetting(string key, string value)
        {
            try { Settings.Instance[key] = value; } catch (Exception) { }
        }

        private static bool CameraEnabled { get { return Setting("astra_camera_on", "true") != "false"; } }
        private static string CameraUrl { get { return Setting("astra_camera_url", DefaultCameraUrl); } }

        // =============================================================== the update (plugin thread, 2 Hz)
        private void Tick()
        {
            var com = Host.comPort;
            if (com == null || com.MAV == null) return;
            var cs = com.MAV.cs;
            ReadMessages(com);
            ReadCompanionTexts(com);
            simulator = DetectSimulator(com);

            bool armed = cs.armed;
            string mode = (cs.mode ?? "").ToUpperInvariant();      // Mission Planner writes "Auto", the autopilot "AUTO"
            lock (sync)
            {
                if (armed && !wasArmed)
                {
                    if (phase == Phase.Landed) ResetFlight();
                    flightStarted = true;
                    AddEvent("ASTRA", "Motors armed: the flight begins", 1);
                }
                if (!armed && wasArmed && flightStarted)
                    AddEvent("ASTRA", "Motors disarmed", 0);
                wasArmed = armed;

                Phase next;
                if (!armed) next = flightStarted ? Phase.Landed : Phase.Waiting;
                else if (returning || mode.StartsWith("RTL") || mode == "LAND" || mode == "SMART_RTL") next = Phase.Return;
                else if (targets.Values.Any(t => t.Dropping || t.Dropped)) next = Phase.Drop;
                else if (targetsChosen) next = Phase.Targets;
                else if (searchDone || photosProcessed >= 0) next = Phase.Process;
                else if (mode == "AUTO" && cs.alt > 10 && cs.wpno >= 2) next = Phase.Search;
                else next = Phase.Takeoff;
                if (armed && next < phase && phase != Phase.Landed) next = phase;     // never step back during a flight
                if (next != phase) AddEvent("ASTRA", "Phase: " + PhaseStrip.Names[(int)next], 1);
                phase = next;
            }

            if (DateTime.Now - lastStatusWrite > TimeSpan.FromSeconds(2)) WriteStatus();
            UpdateUiAsync();
        }

        private static bool DetectSimulator(MAVLinkInterface com)
        {
            string wanted = Setting("astra_sim_buttons", "auto");
            if (wanted == "on") return true;
            if (wanted == "off") return false;
            try { return com.MAV.param.ContainsKey("SIM_SPEEDUP"); } catch (Exception) { return false; }
        }

        private void ResetFlight()
        {
            searchDone = targetsChosen = returning = false;
            photosProcessed = -1;
            foreach (var t in targets.Values)
            {
                t.Detections = 0; t.BestReport = -1; t.HasAim = false; t.Dropping = false; t.Dropped = false;
                t.FirstDetected = DateTime.MinValue; t.DropAt = DateTime.MinValue;
            }
            mapDirty = true;
            AddEvent("ASTRA", "---- a new flight starts ----", 0);
        }

        // =============================================================== status texts -> mission facts
        private void ReadMessages(MAVLinkInterface com)
        {
            foreach (var mav in com.MAVlist.ToList())
            {
                // Only the autopilot's texts here; every other component's texts come through OnStatusText (no doubles).
                if (mav == null || mav.cs == null || mav.cs.messages == null || mav.compid != 1) continue;
                string key = mav.sysid + ":" + mav.compid;
                DateTime last;
                lastMessageTime.TryGetValue(key, out last);
                DateTime newest = last;
                var snapshot = mav.cs.messages.ToArray();
                foreach (var item in snapshot)
                {
                    DateTime time = item.Item1;
                    string text = (item.Item2 ?? "").Trim();
                    if (time < last || text.Length == 0) continue;
                    if (!Remember(key + "|" + time.Ticks + "|" + text)) continue;
                    if (time > newest) newest = time;
                    messagesSeen++;
                    lastMessageAt = DateTime.Now;
                    lock (sync) { HandleText(time, mav.compid, text, com); }
                }
                lastMessageTime[key] = newest;
            }
        }

        private void ReadCompanionTexts(MAVLinkInterface com)
        {
            List<EventItem> batch;
            lock (rawTexts)
            {
                batch = new List<EventItem>(rawTexts);
                rawTexts.Clear();
            }
            foreach (var item in batch)
            {
                companionTexts++;
                messagesSeen++;
                lastMessageAt = DateTime.Now;
                byte compid = 0;
                var parts = item.Source.Split(':');
                if (parts.Length == 2) byte.TryParse(parts[1], out compid);
                lock (sync) { HandleText(item.Time, compid, item.Text, com); }
            }
        }

        private bool Remember(string id)
        {
            if (!seenMessages.Add(id)) return false;
            seenOrder.Enqueue(id);
            while (seenOrder.Count > 3000) seenMessages.Remove(seenOrder.Dequeue());
            return true;
        }

        private static readonly Regex Detected = new Regex(@"Detected (person|tent)", RegexOptions.IgnoreCase);
        private static readonly Regex Processed = new Regex(@"All (\d+) images processed", RegexOptions.IgnoreCase);
        private static readonly Regex Aim = new Regex(@"Aim (person|tent) (-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)", RegexOptions.IgnoreCase);
        private static readonly Regex Over = new Regex(@"Over (person|tent):", RegexOptions.IgnoreCase);
        private static readonly Regex Servo = new Regex(@"Servo (\d+) -> (\d+)", RegexOptions.IgnoreCase);
        private static readonly Regex Tgt = new Regex(@"^TGT ([pt]) (-?\d+\.\d+) (-?\d+\.\d+) (\d+) (\d+)$");
        private static readonly Regex Trouble = new Regex(@"FAILED|failed|timeout|unreadable|No valid targets|No objects detected|PreArm|Bad |Error|error");

        private void HandleText(DateTime time, byte compid, string text, MAVLinkInterface com)
        {
            string source = compid == 1 ? "autopilot" : "flight code";
            int level = Trouble.IsMatch(text) ? 2 : 0;
            Match m;
            if ((m = Detected.Match(text)).Success)
            {
                var t = targets[m.Groups[1].Value.ToLowerInvariant()];
                t.Detections++;
                if (t.FirstDetected == DateTime.MinValue) t.FirstDetected = time;
                level = 1;
            }
            else if ((m = Tgt.Match(text)).Success)
            {
                var t = targets[m.Groups[1].Value == "p" ? "person" : "tent"];
                int confidence = int.Parse(m.Groups[4].Value, CultureInfo.InvariantCulture);
                if (confidence > t.BestReport)
                {
                    t.BestReport = confidence;
                    t.ReportLat = double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
                    t.ReportLon = double.Parse(m.Groups[3].Value, CultureInfo.InvariantCulture);
                }
                if (t.FirstDetected == DateTime.MinValue) t.FirstDetected = time;
                source = "Jetson report";
            }
            else if ((m = Processed.Match(text)).Success)
            {
                photosProcessed = int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture);
                searchDone = true;
            }
            else if (text.StartsWith("Reached last WP", StringComparison.OrdinalIgnoreCase))
            {
                searchDone = true;
            }
            else if ((m = Aim.Match(text)).Success)
            {
                var t = targets[m.Groups[1].Value.ToLowerInvariant()];
                t.HasAim = true;
                t.AimLat = double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
                t.AimLon = double.Parse(m.Groups[3].Value, CultureInfo.InvariantCulture);
                mapDirty = true;
                level = 1;
            }
            else if (text.StartsWith("Targets:", StringComparison.OrdinalIgnoreCase))
            {
                targetsChosen = true;
                level = 1;
            }
            else if ((m = Over.Match(text)).Success)
            {
                targets[m.Groups[1].Value.ToLowerInvariant()].Dropping = true;
            }
            else if ((m = Servo.Match(text)).Success)
            {
                string kind;
                int channel = int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture);
                int pwm = int.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
                if (DropChannels().TryGetValue(channel, out kind) && targets.ContainsKey(kind) && pwm == DropOpenPwm())
                {
                    var t = targets[kind];
                    t.Dropped = true;
                    t.Dropping = false;
                    t.DropAt = time;
                    t.DropLat = com.MAV.cs.lat;
                    t.DropLon = com.MAV.cs.lng;
                    mapDirty = true;
                    AddEvent("ASTRA", "PAYLOAD RELEASED over the " + t.Word, 1);
                }
            }
            else if (text == "RTL" || text.StartsWith("All targets done", StringComparison.OrdinalIgnoreCase)
                     || text.StartsWith("No valid targets", StringComparison.OrdinalIgnoreCase)
                     || text.StartsWith("Divert failed", StringComparison.OrdinalIgnoreCase))
            {
                returning = true;
            }
            AddEvent(source, text, level, time);
        }

        private static Dictionary<int, string> DropChannels()
        {
            var map = new Dictionary<int, string>();
            foreach (var part in Setting("astra_drop_channels", "11:person,13:tent").Split(','))
            {
                var pair = part.Split(':');
                int channel;
                if (pair.Length == 2 && int.TryParse(pair[0].Trim(), out channel)) map[channel] = pair[1].Trim().ToLowerInvariant();
            }
            return map;
        }

        private static int DropOpenPwm()
        {
            int pwm;
            return int.TryParse(Setting("astra_drop_open_pwm", "1400"), out pwm) ? pwm : 1400;
        }

        private void AddEvent(string source, string text, int level)
        {
            AddEvent(source, text, level, DateTime.Now);
        }

        private void AddEvent(string source, string text, int level, DateTime time)
        {
            var item = new EventItem { Time = time, Source = source, Text = text, Level = level };
            lock (sync)
            {
                events.Add(item);
                if (events.Count > 500) events.RemoveRange(0, events.Count - 500);
                pendingEvents.Add(item);
                if (pendingEvents.Count > 500) pendingEvents.RemoveRange(0, pendingEvents.Count - 500);
            }
        }

        // =============================================================== the camera in the HUD (own thread)
        private void CameraLoop()
        {
            while (!stopping)
            {
                string url = CameraUrl;
                if (!CameraEnabled)
                {
                    cameraState = "switched off (HUD right-click > ASTRA camera)";
                    Thread.Sleep(1000);
                    continue;
                }
                if (!Probe(HealthUrl(url)))
                {
                    cameraState = "waiting for the camera stream at " + url;
                    Thread.Sleep(2000);
                    continue;
                }
                try
                {
                    ReadStream(url);
                    cameraState = "the camera stream ended";
                }
                catch (Exception ex)
                {
                    cameraState = "the camera stream stopped (" + ex.Message + ")";
                }
                if (capturing) AddEvent("ASTRA", "Camera picture off: " + cameraState, 0);
                capturing = false;
                ClearHud();
                Thread.Sleep(1000);
            }
        }

        private static string HealthUrl(string url)
        {
            int start = url.IndexOf("//", StringComparison.Ordinal);
            int slash = url.IndexOf('/', start < 0 ? 0 : start + 2);
            return (slash > 0 ? url.Substring(0, slash) : url) + "/health";
        }

        private static bool Probe(string url)
        {
            try
            {
                var request = (HttpWebRequest)WebRequest.Create(url);
                request.Timeout = 1500;
                request.ReadWriteTimeout = 1500;
                request.Proxy = null;
                using (var response = (HttpWebResponse)request.GetResponse())
                    return response.StatusCode == HttpStatusCode.OK;
            }
            catch (Exception)
            {
                return false;
            }
        }

        /// <summary>Read the multipart MJPEG stream and show every complete JPEG (start FF D8 ... end FF D9) in the HUD.</summary>
        private void ReadStream(string url)
        {
            var request = (HttpWebRequest)WebRequest.Create(url);
            request.Timeout = 5000;
            request.ReadWriteTimeout = 5000;          // a silent stream for 5 s ends the read (then it reconnects)
            request.Proxy = null;
            request.KeepAlive = false;
            using (var response = request.GetResponse())
            using (var stream = response.GetResponseStream())
            {
                capturing = true;
                cameraState = "showing " + url;
                AddEvent("ASTRA", "Camera picture on in the HUD (" + url + ")", 1);
                var buffer = new byte[64 * 1024];
                var frame = new MemoryStream();
                bool inFrame = false;
                int previous = -1;
                while (!stopping && CameraEnabled && url == CameraUrl)
                {
                    int count = stream.Read(buffer, 0, buffer.Length);
                    if (count <= 0) break;
                    for (int i = 0; i < count; i++)
                    {
                        byte b = buffer[i];
                        if (!inFrame)
                        {
                            if (previous == 0xFF && b == 0xD8)
                            {
                                inFrame = true;
                                frame.SetLength(0);
                                frame.WriteByte(0xFF);
                                frame.WriteByte(0xD8);
                            }
                        }
                        else
                        {
                            frame.WriteByte(b);
                            if (previous == 0xFF && b == 0xD9)
                            {
                                inFrame = false;
                                ShowFrame(frame.ToArray());
                            }
                            else if (frame.Length > 8 * 1024 * 1024)
                            {
                                inFrame = false;            // not a picture we can use; wait for the next start
                            }
                        }
                        previous = b;
                    }
                }
            }
        }

        private void ShowFrame(byte[] jpeg)
        {
            try
            {
                var hud = FlightData.myhud;
                if (hud == null) return;
                // GDI+ needs the stream for the picture's whole life, so it is not closed here.
                object picture = ImageFromStream.Invoke(null, new object[] { new MemoryStream(jpeg) });
                hud.GetType().GetProperty("bgimage").SetValue(hud, picture, null);
                frames++;
                lastFrameAt = DateTime.Now;
                lock (shownPictures)
                {
                    var disposable = picture as IDisposable;
                    if (disposable != null) shownPictures.Enqueue(disposable);
                    while (shownPictures.Count > 4) shownPictures.Dequeue().Dispose();   // the HUD only draws the newest
                }
            }
            catch (Exception ex)
            {
                cameraState = "a picture could not be shown (" + ex.Message + ")";
            }
        }

        private void ClearHud()
        {
            try
            {
                var hud = FlightData.myhud;
                if (hud != null) hud.GetType().GetProperty("bgimage").SetValue(hud, null, null);
            }
            catch (Exception) { }
            lock (shownPictures)
            {
                while (shownPictures.Count > 0)
                {
                    try { shownPictures.Dequeue().Dispose(); } catch (Exception) { }
                }
            }
        }

        // =============================================================== the ASTRA tab (UI thread)
        private void BuildTab()
        {
            var fd = FlightData.instance;
            if (fd == null) throw new InvalidOperationException("the DATA screen is not ready");
            var tabs = fd.tabControlactions;

            page = new TabPage("ASTRA") { BackColor = Theme.Background, ForeColor = Theme.Text, Padding = new Padding(6) };
            var root = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 6, BackColor = Theme.Background };
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));     // phase strip
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 56));     // what is happening now
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 128));    // readouts
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 116));    // hiker + tent cards
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));     // events
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 92));     // buttons

            strip = new PhaseStrip { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 0, 4) };
            root.Controls.Add(strip, 0, 0);

            var now = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Background, Margin = new Padding(0) };
            statusLine = Ui.Label("Waiting for the drone ...", 13f, true, Theme.Text);
            statusLine.Dock = DockStyle.Top;
            statusLine.Height = 32;
            linkLine = Ui.Label("", 9f, false, Theme.Muted);
            linkLine.Dock = DockStyle.Top;
            linkLine.Height = 20;
            now.Controls.Add(linkLine);
            now.Controls.Add(statusLine);
            root.Controls.Add(now, 0, 1);

            var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 2, BackColor = Theme.Background, Margin = new Padding(0) };
            for (int c = 0; c < 4; c++) grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25));
            for (int r = 0; r < 2; r++) grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            string[] tileNames = { "MODE", "HEIGHT", "SPEED", "BATTERY", "GPS", "HOME", "IN THE AIR", "WAYPOINT" };
            for (int i = 0; i < tileNames.Length; i++) grid.Controls.Add(MakeTile(tileNames[i]), i % 4, i / 4);
            root.Controls.Add(grid, 0, 2);

            var cardRow = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1, BackColor = Theme.Background, Margin = new Padding(0) };
            cardRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            cardRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            cardRow.Controls.Add(MakeCard("person", "HIKER (person)"), 0, 0);
            cardRow.Controls.Add(MakeCard("tent", "TENT"), 1, 0);
            root.Controls.Add(cardRow, 0, 3);

            log = new ListView
            {
                Dock = DockStyle.Fill, View = View.Details, FullRowSelect = true, HeaderStyle = ColumnHeaderStyle.Nonclickable,
                BackColor = Theme.Card, ForeColor = Theme.Text, BorderStyle = BorderStyle.None, Margin = new Padding(0, 4, 0, 4)
            };
            Ui.SetFont(log, 9f, false);
            log.Columns.Add("Time", 70);
            log.Columns.Add("From", 90);
            log.Columns.Add("Message", 400);
            log.Resize += delegate { if (log.Columns.Count == 3) log.Columns[2].Width = Math.Max(200, log.ClientSize.Width - 165); };
            root.Controls.Add(log, 0, 4);

            var buttons = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, BackColor = Theme.Background, Margin = new Padding(0) };
            buttons.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
            buttons.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
            var flight = new FlowLayoutPanel { Dock = DockStyle.Fill, BackColor = Theme.Background, Margin = new Padding(0), WrapContents = false };
            flight.Controls.Add(MakeButton("ARM + START MISSION", Theme.Good, 210, 42, OnStartMission));
            flight.Controls.Add(MakeButton("RETURN HOME (RTL)", Theme.Warn, 190, 42, OnReturnHome));
            buttons.Controls.Add(flight, 0, 0);
            simButtons = new FlowLayoutPanel { Dock = DockStyle.Fill, BackColor = Theme.Background, Margin = new Padding(0), WrapContents = false, Visible = false };
            var simLabel = Ui.Label("SIMULATOR:", 8.5f, true, Theme.Muted);
            simLabel.AutoSize = true;
            simLabel.Margin = new Padding(0, 11, 6, 0);
            simButtons.Controls.Add(simLabel);
            simButtons.Controls.Add(MakeButton("TAKE PHOTO", Theme.Line, 110, 34, delegate { RunSimAction("take_photo", "Take photo"); }));
            simButtons.Controls.Add(MakeButton("CAMERA DOWN", Theme.Line, 125, 34, delegate { RunSimAction("look_down", "Camera down"); }));
            simButtons.Controls.Add(MakeButton("CAMERA FORWARD", Theme.Line, 145, 34, delegate { RunSimAction("look_forward", "Camera forward"); }));
            simButtons.Controls.Add(MakeButton("START OVER", Theme.Line, 115, 34, OnStartOver));
            buttons.Controls.Add(simButtons, 0, 1);
            root.Controls.Add(buttons, 0, 5);

            page.Controls.Add(root);
            tabs.TabPages.Insert(0, page);
            tabs.SelectedTab = page;
        }

        private Control MakeTile(string caption)
        {
            var panel = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Card, Margin = new Padding(3) };
            var title = Ui.Label(caption, 8f, true, Theme.Muted);
            title.Dock = DockStyle.Top;
            title.Height = 20;
            title.Padding = new Padding(6, 4, 0, 0);
            var value = Ui.Label("-", 15f, true, Theme.Text);
            value.Dock = DockStyle.Fill;
            value.Padding = new Padding(6, 0, 0, 0);
            panel.Controls.Add(value);
            panel.Controls.Add(title);
            tiles[caption] = value;
            return panel;
        }

        private Control MakeCard(string kind, string caption)
        {
            var panel = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Card, Margin = new Padding(3), Padding = new Padding(10, 6, 6, 4) };
            var lines = new Label[3];
            for (int i = 2; i >= 0; i--)
            {
                lines[i] = Ui.Label("", 9.5f, false, Theme.Text);
                lines[i].Dock = DockStyle.Top;
                lines[i].Height = 22;
                panel.Controls.Add(lines[i]);
            }
            var title = Ui.Label(caption, 11f, true, Theme.Muted);
            title.Dock = DockStyle.Top;
            title.Height = 26;
            panel.Controls.Add(title);
            var edge = new Panel { Dock = DockStyle.Left, Width = 4, BackColor = Theme.Line };
            panel.Controls.Add(edge);
            cards[kind] = new[] { title, lines[0], lines[1], lines[2] };
            cardEdges[kind] = edge;
            return panel;
        }

        private static Button MakeButton(string text, Color color, int width, int height, EventHandler click)
        {
            var b = new Button
            {
                Text = text, Width = width, Height = height, FlatStyle = FlatStyle.Flat, BackColor = color, ForeColor = Color.White,
                Margin = new Padding(0, 3, 8, 3), Cursor = Cursors.Hand
            };
            Ui.SetFont(b, 9f, true);
            b.FlatAppearance.BorderSize = 0;
            b.Click += click;
            return b;
        }

        private void UpdateUiAsync()
        {
            if (page == null || !page.IsHandleCreated) return;
            try { page.BeginInvoke((MethodInvoker)UpdateUi); } catch (Exception) { }
        }

        private void UpdateUi()
        {
            try
            {
                var com = Host.comPort;
                var cs = com.MAV.cs;
                List<EventItem> fresh;
                Phase shown;
                lock (sync)
                {
                    fresh = new List<EventItem>(pendingEvents);
                    pendingEvents.Clear();
                    shown = phase;
                }
                strip.Current = (int)shown;
                SetText(statusLine, Sentence(shown, cs));
                double sinceMessage = lastMessageAt == DateTime.MinValue ? -1 : (DateTime.Now - lastMessageAt).TotalSeconds;
                bool linked = cs.connected || (com.BaseStream != null && com.BaseStream.IsOpen);
                SetText(linkLine, (linked ? "Telemetry connected" : "No telemetry link")
                    + "   |   camera: " + cameraState + (capturing ? " (" + frames + " pictures)" : "")
                    + (sinceMessage >= 0 ? "   |   last message " + (int)sinceMessage + " s ago" : ""));

                SetTile("MODE", string.IsNullOrEmpty(cs.mode) ? "-" : cs.mode + (cs.armed ? "  ARMED" : ""), cs.armed ? Theme.Warn : Theme.Text);
                SetTile("HEIGHT", cs.alt.ToString("0.0", CultureInfo.InvariantCulture) + " m", Theme.Text);
                SetTile("SPEED", cs.groundspeed.ToString("0.0", CultureInfo.InvariantCulture) + " m/s", Theme.Text);
                SetTile("BATTERY", (cs.battery_remaining > 0 ? cs.battery_remaining + " %  " : "") + cs.battery_voltage.ToString("0.0", CultureInfo.InvariantCulture) + " V",
                    cs.battery_remaining > 0 && cs.battery_remaining < 30 ? Theme.Bad : Theme.Text);
                SetTile("GPS", (cs.gpsstatus >= 3 ? "3D fix" : (cs.gpsstatus >= 2 ? "2D fix" : "no fix")) + "  " + (int)cs.satcount + " sats", cs.gpsstatus >= 3 ? Theme.Text : Theme.Bad);
                SetTile("HOME", cs.DistToHome.ToString("0", CultureInfo.InvariantCulture) + " m", Theme.Text);
                SetTile("IN THE AIR", TimeSpan.FromSeconds(Math.Max(0, cs.timeInAir)).ToString(@"m\:ss"), Theme.Text);
                int count = com.MAV.wps == null ? 0 : com.MAV.wps.Count;
                SetTile("WAYPOINT", count > 0 ? ((int)cs.wpno) + " of " + Math.Max(0, count - 1) : "no mission", count > 0 ? Theme.Text : Theme.Warn);

                foreach (var kind in new[] { "person", "tent" })
                {
                    string detections, target, payload;
                    Color edge, title;
                    lock (sync)
                    {
                        var t = targets[kind];
                        detections = t.Detections > 0 || t.BestReport >= 0
                            ? (t.Detections > 0 ? "Detected " + t.Detections + " time" + (t.Detections == 1 ? "" : "s") : "Reported")
                              + (t.BestReport >= 0 ? "  (best " + t.BestReport + "% at " + t.ReportLat.ToString("0.00000", CultureInfo.InvariantCulture) + ", " + t.ReportLon.ToString("0.00000", CultureInfo.InvariantCulture) + ")"
                                                   : "  (first at " + t.FirstDetected.ToString("HH:mm:ss") + ")")
                            : "Not detected yet";
                        target = t.HasAim
                            ? "Target: " + t.AimLat.ToString("0.0000000", CultureInfo.InvariantCulture) + ", " + t.AimLon.ToString("0.0000000", CultureInfo.InvariantCulture)
                            : (photosProcessed >= 0 ? "No target chosen" : "Target: chosen after the search");
                        payload = t.Dropped
                            ? "Payload RELEASED at " + t.DropAt.ToString("HH:mm:ss") + "  (" + t.DropLat.ToString("0.000000", CultureInfo.InvariantCulture) + ", " + t.DropLon.ToString("0.000000", CultureInfo.InvariantCulture) + ")"
                            : (t.Dropping ? "Over the target: dropping ..." : "Payload on board");
                        edge = t.Dropped ? Theme.Good : (t.HasAim ? Theme.Accent : (t.Detections > 0 || t.BestReport >= 0 ? Theme.Warn : Theme.Line));
                        title = t.Dropped ? Theme.DoneText : (t.HasAim ? Theme.Accent : (t.Detections > 0 ? Theme.Warn : Theme.Muted));
                    }
                    var labels = cards[kind];
                    SetText(labels[1], detections);
                    SetText(labels[2], target);
                    SetText(labels[3], payload);
                    labels[0].ForeColor = title;
                    labels[3].ForeColor = payload.StartsWith("Payload RELEASED") ? Theme.DoneText : Theme.Text;
                    if (cardEdges[kind].BackColor != edge) cardEdges[kind].BackColor = edge;
                }

                if (fresh.Count > 0)
                {
                    log.BeginUpdate();
                    foreach (var item in fresh)
                    {
                        var row = new ListViewItem(new[] { item.Time.ToString("HH:mm:ss"), item.Source, item.Text });
                        row.ForeColor = item.Level == 2 ? Theme.Warn : (item.Level == 1 ? Theme.DoneText : Theme.Text);
                        log.Items.Insert(0, row);
                    }
                    while (log.Items.Count > 300) log.Items.RemoveAt(log.Items.Count - 1);
                    log.EndUpdate();
                }

                if (simButtons.Visible != simulator) simButtons.Visible = simulator;
                if (mapDirty) RefreshMap();
            }
            catch (Exception ex)
            {
                lastError = "screen: " + ex.Message;
            }
        }

        private static void SetText(Label label, string text)
        {
            if (label.Text != text) label.Text = text;
        }

        private void SetTile(string name, string text, Color color)
        {
            Label label;
            if (!tiles.TryGetValue(name, out label)) return;
            SetText(label, text);
            if (label.ForeColor != color) label.ForeColor = color;
        }

        private string Sentence(Phase p, CurrentState cs)
        {
            lock (sync)
            {
                switch (p)
                {
                    case Phase.Waiting:
                        return cs.connected || cs.mode != null ? "On the ground, disarmed. Load the mission (PLAN), then press ARM + START MISSION." : "Waiting for the drone's telemetry ...";
                    case Phase.Takeoff:
                        return "Taking off: " + cs.alt.ToString("0", CultureInfo.InvariantCulture) + " m";
                    case Phase.Search:
                        return "Searching: waypoint " + (int)cs.wpno + ", " + cs.alt.ToString("0", CultureInfo.InvariantCulture) + " m, "
                            + cs.groundspeed.ToString("0.0", CultureInfo.InvariantCulture) + " m/s. Detections so far: hiker "
                            + targets["person"].Detections + ", tent " + targets["tent"].Detections;
                    case Phase.Process:
                        return photosProcessed >= 0 ? "Search finished: " + photosProcessed + " photos processed, choosing the targets" : "Search finished: holding while the detector finishes the photos";
                    case Phase.Targets:
                        return "Flying to the targets";
                    case Phase.Drop:
                        var over = targets.Values.FirstOrDefault(t => t.Dropping && !t.Dropped);
                        if (over != null) return "Over the " + over.Word + ": dropping the payload";
                        return "Payloads released: " + string.Join(", ", targets.Values.Where(t => t.Dropped).Select(t => t.Word).ToArray());
                    case Phase.Return:
                        return "Returning home: " + cs.DistToHome.ToString("0", CultureInfo.InvariantCulture) + " m to go, " + cs.alt.ToString("0", CultureInfo.InvariantCulture) + " m high";
                    default:
                        return "Landed and disarmed. Flight time " + TimeSpan.FromSeconds(Math.Max(0, cs.timeInAir)).ToString(@"m\:ss") + ".";
                }
            }
        }

        // =============================================================== the map
        private void AddMapOverlay()
        {
            overlay = new GMapOverlay("astra");
            Host.FDGMapControl.Overlays.Add(overlay);
        }

        private void RefreshMap()
        {
            mapDirty = false;
            if (overlay == null) return;
            overlay.Markers.Clear();
            lock (sync)
            {
                foreach (var t in targets.Values)
                {
                    string who = t.Kind == "person" ? "Hiker" : "Tent";
                    if (t.HasAim)
                    {
                        var marker = new GMarkerGoogle(new PointLatLng(t.AimLat, t.AimLon), t.Kind == "person" ? GMarkerGoogleType.red_dot : GMarkerGoogleType.orange_dot);
                        marker.ToolTipText = who + " target";
                        marker.ToolTipMode = MarkerTooltipMode.Always;
                        overlay.Markers.Add(marker);
                    }
                    if (t.Dropped && t.DropLat != 0)
                    {
                        var marker = new GMarkerGoogle(new PointLatLng(t.DropLat, t.DropLon), GMarkerGoogleType.green_small);
                        marker.ToolTipText = who + " payload released " + t.DropAt.ToString("HH:mm:ss");
                        marker.ToolTipMode = MarkerTooltipMode.OnMouseOver;
                        overlay.Markers.Add(marker);
                    }
                }
            }
        }

        // =============================================================== the HUD menu
        private void AddHudMenu()
        {
            var root = new ToolStripMenuItem("ASTRA camera");
            var toggle = new ToolStripMenuItem("Show the drone camera in the HUD") { CheckOnClick = true, Checked = CameraEnabled };
            toggle.CheckedChanged += delegate { SaveSetting("astra_camera_on", toggle.Checked ? "true" : "false"); };
            var address = new ToolStripMenuItem("Camera address ...");
            address.Click += delegate
            {
                string url = CameraUrl;
                if (MissionPlanner.Controls.InputBox.Show("ASTRA camera", "MJPEG address of the drone camera", ref url) == DialogResult.OK && !string.IsNullOrEmpty(url))
                    SaveSetting("astra_camera_url", url.Trim());        // the camera thread notices and reconnects
            };
            root.DropDownItems.Add(toggle);
            root.DropDownItems.Add(address);
            Host.FDMenuHud.Items.Add(root);
        }

        // =============================================================== buttons
        private void OnStartMission(object sender, EventArgs e)
        {
            var com = Host.comPort;
            int count = com.MAV.wps == null ? 0 : com.MAV.wps.Count;
            if (count < 2)
            {
                MessageBox.Show("There is no mission on the drone yet.\n\nLoad it in PLAN (Load WP File, then Write WPs), or start the simulator with demo.sh, then try again.",
                    "ASTRA", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            if (MessageBox.Show("Arm the motors and start the mission (AUTO)?\n\nThe drone will take off and fly the " + (count - 1) + " mission items.",
                    "ASTRA: start the mission", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes)
                return;
            ThreadPool.QueueUserWorkItem(delegate
            {
                try
                {
                    AddEvent("ASTRA", "Start: GUIDED, arm, AUTO ...", 0);
                    byte sysid = com.MAV.sysid, compid = com.MAV.compid;
                    com.setMode(sysid, compid, "GUIDED");
                    Thread.Sleep(500);
                    bool armed = com.doARM(sysid, compid, true, false);
                    if (!armed)
                    {
                        AddEvent("ASTRA", "The autopilot refused to arm (see its PreArm messages)", 2);
                        return;
                    }
                    Thread.Sleep(1000);
                    com.setMode(sysid, compid, "AUTO");
                    AddEvent("ASTRA", "Mission started (AUTO)", 1);
                }
                catch (Exception ex)
                {
                    AddEvent("ASTRA", "Start failed: " + ex.Message, 2);
                }
            });
        }

        private void OnReturnHome(object sender, EventArgs e)
        {
            if (MessageBox.Show("Fly back to the launch point and land (RTL)?", "ASTRA: return home", MessageBoxButtons.YesNo, MessageBoxIcon.Question) != DialogResult.Yes)
                return;
            var com = Host.comPort;
            ThreadPool.QueueUserWorkItem(delegate
            {
                try
                {
                    com.setMode(com.MAV.sysid, com.MAV.compid, "RTL");
                    AddEvent("ASTRA", "Return home (RTL) requested", 1);
                }
                catch (Exception ex)
                {
                    AddEvent("ASTRA", "RTL failed: " + ex.Message, 2);
                }
            });
        }

        private void OnStartOver(object sender, EventArgs e)
        {
            if (MessageBox.Show("Put the simulation back to the start?\n\nA flying drone lands first; then a fresh drone with both payloads stands on the start point and the mission is uploaded again. Press ARM + START MISSION afterwards.",
                    "ASTRA: start over (simulator)", MessageBoxButtons.YesNo, MessageBoxIcon.Question) != DialogResult.Yes)
                return;
            RunSimAction("start_over", "Start over");
        }

        private void RunSimAction(string action, string label)
        {
            string distro = Setting("astra_wsl_distro", "Ubuntu-22.04");
            string repo = Setting("astra_wsl_repo", "~/ODLC_Machine_Inferencing_System_2025-2026");
            AddEvent("ASTRA", label + " ...", 0);
            ThreadPool.QueueUserWorkItem(delegate
            {
                try
                {
                    string windir = Environment.GetEnvironmentVariable("WINDIR") ?? @"C:\Windows";
                    // A 32-bit program sees System32 redirected; "Sysnative" is the way to the real 64-bit wsl.exe.
                    string wsl = Environment.Is64BitProcess || !Environment.Is64BitOperatingSystem
                        ? Path.Combine(windir, @"System32\wsl.exe") : Path.Combine(windir, @"Sysnative\wsl.exe");
                    var info = new ProcessStartInfo(wsl, "-d " + distro + " -- bash -lc \"bash " + repo + "/simulation/scripts/mp_actions.sh " + action + "\"")
                    {
                        UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true
                    };
                    using (var process = Process.Start(info))
                    {
                        string output = process.StandardOutput.ReadToEnd() + process.StandardError.ReadToEnd();
                        process.WaitForExit(15 * 60 * 1000);
                        string lastLine = output.Split(new[] { '\n' }).Select(s => s.Trim()).Where(s => s.Length > 0).LastOrDefault() ?? "";
                        AddEvent("ASTRA", label + (process.ExitCode == 0 ? ": done" : ": FAILED") + (lastLine.Length > 0 ? " - " + lastLine : ""), process.ExitCode == 0 ? 1 : 2);
                    }
                }
                catch (Exception ex)
                {
                    AddEvent("ASTRA", label + ": could not run WSL (" + ex.Message + ")", 2);
                }
            });
        }

        // =============================================================== the status file
        private void WriteStatus()
        {
            lastStatusWrite = DateTime.Now;
            try
            {
                var sb = new StringBuilder();
                sb.Append("{\n");
                lock (sync)
                {
                    Field(sb, "plugin_version", PluginVersion, false);
                    Field(sb, "written", DateTime.Now.ToString("o"), false);
                    Field(sb, "phase", PhaseStrip.Names[(int)phase], false);
                    Field(sb, "tab_built", page != null ? "true" : "false", true);
                    Field(sb, "camera_state", cameraState, false);
                    Field(sb, "camera_pictures", frames.ToString(CultureInfo.InvariantCulture), true);
                    Field(sb, "camera_last_picture_age_s", lastFrameAt == DateTime.MinValue ? "null" : (DateTime.Now - lastFrameAt).TotalSeconds.ToString("0.0", CultureInfo.InvariantCulture), true);
                    Field(sb, "messages_seen", messagesSeen.ToString(CultureInfo.InvariantCulture), true);
                    Field(sb, "companion_texts", companionTexts.ToString(CultureInfo.InvariantCulture), true);
                    Field(sb, "status_text_subscription", statustextSubscription >= 0 ? "true" : "false", true);
                    Field(sb, "simulator", simulator ? "true" : "false", true);
                    Field(sb, "photos_processed", photosProcessed.ToString(CultureInfo.InvariantCulture), true);
                    foreach (var t in targets.Values)
                    {
                        Field(sb, t.Kind + "_detections", t.Detections.ToString(CultureInfo.InvariantCulture), true);
                        Field(sb, t.Kind + "_target", t.HasAim ? t.AimLat.ToString("0.0000000", CultureInfo.InvariantCulture) + "," + t.AimLon.ToString("0.0000000", CultureInfo.InvariantCulture) : "", false);
                        Field(sb, t.Kind + "_released", t.Dropped ? t.DropAt.ToString("HH:mm:ss") : "", false);
                    }
                    Field(sb, "events", events.Count.ToString(CultureInfo.InvariantCulture), true);
                    Field(sb, "last_event", events.Count > 0 ? events[events.Count - 1].Text : "", false);
                }
                Field(sb, "last_error", lastError, false);
                sb.Append("  \"end\": true\n}\n");
                string folder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Mission Planner");
                Directory.CreateDirectory(folder);
                File.WriteAllText(Path.Combine(folder, "astra_plugin_status.json"), sb.ToString());
            }
            catch (Exception ex)
            {
                lastError = "status file: " + ex.Message;
            }
        }

        private static void Field(StringBuilder sb, string name, string value, bool raw)
        {
            sb.Append("  \"").Append(name).Append("\": ");
            if (raw) sb.Append(value);
            else sb.Append('"').Append((value ?? "").Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\r", " ").Replace("\n", " ")).Append('"');
            sb.Append(",\n");
        }
    }
}
