# SIYI ZR10 Code Control --- Quick How-To

This guide covers five ZR10 camera functions using the **SIYI Gimbal
SDK**:

1.  Take Image
2.  Auto Focus
3.  Manual Focus
4.  Zoom In
5.  Video Recording

The commands below come from the ZR10 User Manual's **UART / UDP Control
(SIYI Gimbal SDK)** section.

------------------------------------------------------------------------

## 1. Take Image

**Command ID:** `0x0C`

Use the camera function command with:

``` text
func_type = 0
```

Meaning:

``` text
0 = Take a picture
```

Example packet from the manual:

``` text
55 66 01 01 00 00 00 0C 00 34 CE
```

Conceptual C++ use:

``` cpp
takePicture();
```

Your `takePicture()` function should construct and transmit the `0x0C`
command with `func_type = 0`.

------------------------------------------------------------------------

## 2. Auto Focus

**Command ID:** `0x04`

Use:

``` text
auto_focus = 1
```

Meaning:

``` text
1 = Start auto focus once
```

The command also supports `touch_x` and `touch_y` coordinates for
selecting a focus location in the video image.

Example simple autofocus packet from the manual:

``` text
55 66 01 01 00 00 00 04 01 BC 57
```

Conceptual C++ use:

``` cpp
autoFocus();
```

------------------------------------------------------------------------

## 3. Manual Focus

**Command ID:** `0x06`

The `focus` value controls the direction of manual focusing:

``` text
focus =  1  -> Long shot
focus =  0  -> Stop manual focus
focus = -1  -> Close shot
```

Conceptual C++ use:

``` cpp
manualFocus(1);   // Focus toward long shot
manualFocus(0);   // Stop focusing
manualFocus(-1);  // Focus toward close shot
```

Because this is a start/stop style control, send `0` when you want
manual focus movement to stop.

Example:

``` cpp
manualFocus(1);

std::this_thread::sleep_for(
    std::chrono::milliseconds(300)
);

manualFocus(0);
```

------------------------------------------------------------------------

## 4. Zoom In

**Command ID:** `0x05`

The `zoom` value controls zoom direction:

``` text
zoom =  1  -> Start zooming in
zoom =  0  -> Stop zooming
zoom = -1  -> Start zooming out
```

Example zoom-in packet from the manual:

``` text
55 66 01 01 00 00 00 05 01 8D 64
```

Conceptual C++ use:

``` cpp
zoom(1);  // Start zooming in
zoom(0);  // Stop zooming
```

For example, to zoom in briefly:

``` cpp
zoom(1);

std::this_thread::sleep_for(
    std::chrono::milliseconds(500)
);

zoom(0);
```

The important part is sending `zoom(0)` when the desired zoom has been
reached.

------------------------------------------------------------------------

## 5. Video Recording

**Command ID:** `0x0C`

Use:

``` text
func_type = 2
```

Meaning:

``` text
2 = Start / Stop Recording
```

The same command toggles recording on and off.

Example packet from the manual:

``` text
55 66 01 01 00 00 00 0C 02 76 EE
```

Conceptual C++ use:

``` cpp
toggleRecording();  // Start recording

// ...

toggleRecording();  // Stop recording
```

------------------------------------------------------------------------

## Quick Reference

  Function         CMD_ID Data
  -------------- -------- --------------------------------
  Take Image       `0x0C` `func_type = 0`
  Auto Focus       `0x04` `auto_focus = 1`
  Manual Focus     `0x06` `1` long, `0` stop, `-1` close
  Zoom In          `0x05` `1` zoom in, `0` stop
  Video            `0x0C` `func_type = 2`

## SDK Packet Structure

ZR10 SDK commands are transmitted using the SIYI packet structure:

``` text
STX | CTRL | DATA_LEN | SEQ | CMD_ID | DATA | CRC16
```

The examples above show complete packets supplied in the ZR10 manual
where available. In a C++ program, functions such as `zoom()` or
`takePicture()` would construct the appropriate SDK packet,
calculate/include its CRC16, and transmit it to the ZR10 using the
selected SDK communication interface.
