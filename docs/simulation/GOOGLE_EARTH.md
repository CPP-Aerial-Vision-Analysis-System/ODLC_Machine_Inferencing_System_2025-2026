<!-- PURPOSE: describe using Google Earth/Maps as an external location comparison while keeping their imagery and 3D data out of the simulator and public repository. -->

# Compare the field with Google Earth

This project's Google workflow is a live browser/Earth Pro comparison. Keep Google imagery, extracted
3D models, map tiles and offline datasets out of Gazebo assets and the public repo. Build simulator
terrain/textures from independently licensed sources with recorded provenance; see [WORLDS](WORLDS.md).

Google's [Maps additional terms](https://www.google.com/help/terms_maps/) and
[Earth additional terms](https://maps.google.com/intl/en_all/help/terms_maps-earth/) restrict uses such
as bulk downloading. Their content is not an open dataset merely because it is visible in the viewer.
The repo's comparison-only rule avoids depending on content redistribution rights.

## Compare a location

1. Read the selected world's home/search coordinates from its config or `world_info.py` output.
2. Enter latitude, longitude into Google Earth Pro or the Maps search box. For the current synthetic
   worlds, the configured home is `36.2143000, -96.0068000`; check the selected world's actual config.
3. Compare field shape, roads and general orientation alongside the simulator's mission map. Record
   the imagery date and the source of any independently obtained survey boundary.
4. Keep simulator measurements tied to its source CRS, elevation datum and truth data. A visual match
   to Google's globe does not establish survey accuracy or correct AGL altitude.

Google Earth and the simulator may use imagery from different dates and different terrain processing.
Use the comparison to identify questions, then resolve geometry/elevation through the licensed source
data rather than extracting Google's pixels or meshes.

## Sharing and future integrations

For permitted presentations, review Google's
[Geo Guidelines](https://about.google/brand-resource-center/products-and-services/geo-guidelines/)
for the specific medium and keep Google/provider attribution visible. Do not crop attribution out of
a comparison image. This does not authorize putting that image into a simulator texture package.

A future Open in Maps link can open a coordinate in the normal website. An embedded live map would
need a supported API, the owner's own credentials/billing where required, and compliance with the API's
terms. No API key belongs in Git, and no embedded map or map-export button is currently implemented here.
