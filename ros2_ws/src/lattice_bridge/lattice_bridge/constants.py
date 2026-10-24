"""
lattice_bridge/constants.py -- PURPOSE: the ONE place where every fixed word and number of the Lattice
bridge lives: the integration name, the Lattice enum words (templates, dispositions, environments, geo
types, health states), the "platform type" names of our drone and our targets, the size and time limits,
the radio-message limits and the default settings of the target clusterer. Every other module imports
from here, so a word that must change (for example after checking the Lattice Schema Registry) changes
in exactly one line, and the tests can compare the whole package against one list.

Where the words come from: the Lattice REST reference "PublishEntity"
(https://developer.anduril.com/reference/rest/entities/publish-entity), read on 2026-10-24, and the saved
pages in the repo root (Entities overview, Quickstart). The old address docs.anduril.com now redirects to
developer.anduril.com. The *_INVALID values (the zero value of each enum) are left out on purpose: a
published entity must never carry one.

No code runs here and nothing is imported except datetime, so importing this module is free and safe.
"""

from datetime import timedelta

# ---------------------------------------------------------------------------------------------------
# Who we are, as Lattice sees us (provenance component)
# ---------------------------------------------------------------------------------------------------

#: provenance.integrationName on every entity we publish. Lattice uses it to show which integration made
#: an entity, so it never changes between missions.
INTEGRATION_NAME = 'astra_odlc'

#: provenance.dataType per kind of entity (a free-text label in Lattice; ours are fixed here).
DATA_TYPE_UAV = 'uav_telemetry'
DATA_TYPE_TARGET = 'odlc_detection'
DATA_TYPE_AREA = 'mission_area'
DATA_TYPE_DROP = 'payload_drop'
DATA_TYPES = frozenset({DATA_TYPE_UAV, DATA_TYPE_TARGET, DATA_TYPE_AREA, DATA_TYPE_DROP})

# ---------------------------------------------------------------------------------------------------
# Lattice enum words (JSON uses the full word, exactly as written here)
# ---------------------------------------------------------------------------------------------------

#: ontology.template: which of the three entity shapes this is.
TEMPLATE_ASSET = 'TEMPLATE_ASSET'          # a vehicle we control (our drone)
TEMPLATE_TRACK = 'TEMPLATE_TRACK'          # something a sensor saw (hiker, tent)
TEMPLATE_GEO = 'TEMPLATE_GEO'              # a shape on the map (search area, boundary, drop point)
TEMPLATE_SENSOR_POINT_OF_INTEREST = 'TEMPLATE_SENSOR_POINT_OF_INTEREST'
TEMPLATE_SIGNAL_OF_INTEREST = 'TEMPLATE_SIGNAL_OF_INTEREST'
TEMPLATES = frozenset({
    TEMPLATE_ASSET, TEMPLATE_TRACK, TEMPLATE_GEO,
    TEMPLATE_SENSOR_POINT_OF_INTEREST, TEMPLATE_SIGNAL_OF_INTEREST,
})

#: milView.disposition: whose side the entity is on.
DISPOSITION_UNKNOWN = 'DISPOSITION_UNKNOWN'
DISPOSITION_FRIENDLY = 'DISPOSITION_FRIENDLY'
DISPOSITION_HOSTILE = 'DISPOSITION_HOSTILE'
DISPOSITION_SUSPICIOUS = 'DISPOSITION_SUSPICIOUS'
DISPOSITION_ASSUMED_FRIENDLY = 'DISPOSITION_ASSUMED_FRIENDLY'
DISPOSITION_NEUTRAL = 'DISPOSITION_NEUTRAL'
DISPOSITION_PENDING = 'DISPOSITION_PENDING'
DISPOSITIONS = frozenset({
    DISPOSITION_UNKNOWN, DISPOSITION_FRIENDLY, DISPOSITION_HOSTILE, DISPOSITION_SUSPICIOUS,
    DISPOSITION_ASSUMED_FRIENDLY, DISPOSITION_NEUTRAL, DISPOSITION_PENDING,
})

#: milView.environment: where the entity is.
ENVIRONMENT_UNKNOWN = 'ENVIRONMENT_UNKNOWN'
ENVIRONMENT_AIR = 'ENVIRONMENT_AIR'
ENVIRONMENT_SURFACE = 'ENVIRONMENT_SURFACE'
ENVIRONMENT_SUB_SURFACE = 'ENVIRONMENT_SUB_SURFACE'
ENVIRONMENT_LAND = 'ENVIRONMENT_LAND'
ENVIRONMENT_SPACE = 'ENVIRONMENT_SPACE'
ENVIRONMENTS = frozenset({
    ENVIRONMENT_UNKNOWN, ENVIRONMENT_AIR, ENVIRONMENT_SURFACE, ENVIRONMENT_SUB_SURFACE,
    ENVIRONMENT_LAND, ENVIRONMENT_SPACE,
})

#: geoDetails.type: what kind of map shape a geo-entity is.
GEO_TYPE_GENERAL = 'GEO_TYPE_GENERAL'
GEO_TYPE_HAZARD = 'GEO_TYPE_HAZARD'
GEO_TYPE_EMERGENCY = 'GEO_TYPE_EMERGENCY'
GEO_TYPE_ENGAGEMENT_ZONE = 'GEO_TYPE_ENGAGEMENT_ZONE'
GEO_TYPE_CONTROL_AREA = 'GEO_TYPE_CONTROL_AREA'
GEO_TYPE_BULLSEYE = 'GEO_TYPE_BULLSEYE'
GEO_TYPE_ACM = 'GEO_TYPE_ACM'
GEO_TYPES = frozenset({
    GEO_TYPE_GENERAL, GEO_TYPE_HAZARD, GEO_TYPE_EMERGENCY, GEO_TYPE_ENGAGEMENT_ZONE,
    GEO_TYPE_CONTROL_AREA, GEO_TYPE_BULLSEYE, GEO_TYPE_ACM,
})

#: health.connectionStatus / health.healthStatus / health.components[].health
CONNECTION_STATUS_ONLINE = 'CONNECTION_STATUS_ONLINE'
CONNECTION_STATUS_OFFLINE = 'CONNECTION_STATUS_OFFLINE'
CONNECTION_STATUSES = frozenset({CONNECTION_STATUS_ONLINE, CONNECTION_STATUS_OFFLINE})

HEALTH_STATUS_HEALTHY = 'HEALTH_STATUS_HEALTHY'
HEALTH_STATUS_WARN = 'HEALTH_STATUS_WARN'
HEALTH_STATUS_FAIL = 'HEALTH_STATUS_FAIL'
HEALTH_STATUS_OFFLINE = 'HEALTH_STATUS_OFFLINE'
HEALTH_STATUS_NOT_READY = 'HEALTH_STATUS_NOT_READY'
HEALTH_STATUSES = frozenset({
    HEALTH_STATUS_HEALTHY, HEALTH_STATUS_WARN, HEALTH_STATUS_FAIL, HEALTH_STATUS_OFFLINE,
    HEALTH_STATUS_NOT_READY,
})

#: The shapes a geoShape may hold (exactly one is allowed per entity).
GEO_SHAPE_KINDS = frozenset({'point', 'line', 'polygon', 'ellipse', 'ellipsoid'})

#: Every top-level key the PublishEntity request body knows (39 on 2026-10-24). validate_entity() reports
#: any other key, which catches typos and snake_case slips (platform_type instead of platformType).
ENTITY_TOP_LEVEL_KEYS = frozenset({
    'entityId', 'description', 'isLive', 'createdTime', 'expiryTime', 'noExpiry', 'status',
    'location', 'locationUncertainty', 'kinematics', 'geoShape', 'geoDetails', 'aliases', 'tracked',
    'correlation', 'milView', 'ontology', 'sensors', 'payloads', 'powerState', 'provenance',
    'overrides', 'indicators', 'targetPriority', 'signal', 'transponderCodes', 'dataClassification',
    'taskCatalog', 'media', 'relationships', 'visualDetails', 'dimensions', 'routeDetails',
    'schedules', 'health', 'groupDetails', 'supplies', 'orbit', 'symbology',
})

# ---------------------------------------------------------------------------------------------------
# What we publish: platform types and the fixed choices for each kind of entity
# ---------------------------------------------------------------------------------------------------

#: ontology.platformType is free text in the REST reference ("high-level type in natural language"), but
#: the Lattice Schema Registry may define preferred names. CONFIRM THESE THREE IN THE REGISTRY before the
#: sandbox phase (LATTICE_PLAN.md phase 3). They are defined only here; change them only here.
PLATFORM_TYPE_UAV = 'UAV'          # appears in the saved Entities overview page as the UAV example
PLATFORM_TYPE_PERSON = 'Person'    # TO CONFIRM
PLATFORM_TYPE_TENT = 'Tent'        # TO CONFIRM

#: The two kinds of target the competition asks for. "person" is the hiker (a mannequin); "tent" is the tent.
KIND_PERSON = 'person'
KIND_TENT = 'tent'
TARGET_KINDS = (KIND_PERSON, KIND_TENT)
PLATFORM_TYPE_BY_KIND = {KIND_PERSON: PLATFORM_TYPE_PERSON, KIND_TENT: PLATFORM_TYPE_TENT}
#: Words for humans (descriptions and display names), by kind.
LABEL_BY_KIND = {KIND_PERSON: 'Person', KIND_TENT: 'Tent'}

#: milView of our drone: ours (friendly), in the air.
UAV_DISPOSITION = DISPOSITION_FRIENDLY
UAV_ENVIRONMENT = ENVIRONMENT_AIR
#: milView of a target: nobody's enemy (neutral), on the ground. ENVIRONMENT_LAND also exists in the live
#: reference and may suit a person better; the plan (section 3) says SURFACE, so SURFACE it is. One line to change.
TARGET_DISPOSITION = DISPOSITION_NEUTRAL
TARGET_ENVIRONMENT = ENVIRONMENT_SURFACE

#: geoDetails.type for each kind of map shape we publish.
GEO_TYPE_SEARCH_AREA = GEO_TYPE_GENERAL
GEO_TYPE_FLIGHT_BOUNDARY = GEO_TYPE_CONTROL_AREA
GEO_TYPE_DROP_POINT = GEO_TYPE_GENERAL

# ---------------------------------------------------------------------------------------------------
# Time limits
# ---------------------------------------------------------------------------------------------------

#: Lattice rule: expiryTime must be in the future and LESS than 30 days away.
LATTICE_MAX_EXPIRY = timedelta(days=30)
#: Our own, safer limit: 29 days. A laptop clock that runs a little slow would otherwise push a 29d23h
#: expiry over the 30-day line in Lattice's eyes.
MAX_EXPIRY = timedelta(days=29)
#: The drone entity lives 2 minutes after the last update, so it vanishes soon after the link dies
#: (LATTICE_PLAN.md section 3). Every update pushes the expiry forward again.
UAV_TTL_S = 120.0
#: Recommended margin after the planned end of the mission for targets (they stay visible for debriefing).
TARGET_EXPIRY_MARGIN_S = 6 * 3600.0
#: Recommended lifetime of search areas, the flight boundary and drop points, counted from publishing.
AREA_TTL_S = 12 * 3600.0
#: A time stamp this far ahead of "now" is accepted (clocks of the Jetson, laptop and GPS differ a little).
FUTURE_TOLERANCE_S = 60.0

# ---------------------------------------------------------------------------------------------------
# Size and range limits (our own safety nets; Lattice does not publish most of them)
# ---------------------------------------------------------------------------------------------------

MAX_ENTITY_ID_LEN = 128
MAX_NAME_LEN = 80
MAX_DESCRIPTION_LEN = 300
MAX_MODE_LEN = 24
MAX_POLYGON_VERTICES = 200
MIN_POLYGON_VERTICES = 3

#: Height limits (metres) that stop garbage such as 4294967 or -1e9 from becoming an altitude.
ALT_MIN_M = -1000.0
ALT_MAX_M = 100000.0
#: Speed limit (m/s) for the optional velocity.
SPEED_MAX_MPS = 1000.0

#: "Link health" of the telemetry radio, from the age of the newest telemetry message.
LINK_OK_MAX_AGE_S = 3.0       # newer than this: HEALTHY
LINK_WARN_MAX_AGE_S = 10.0    # newer than this: WARN; older: FAIL
#: Battery state of charge in percent.
BATTERY_WARN_BELOW_PERCENT = 30.0
BATTERY_FAIL_BELOW_PERCENT = 15.0

#: tracked.trackQualityWrapper is a score from 0 to 15 in Lattice; our 0..1 confidence is mapped onto it.
TRACK_QUALITY_MAX = 15

# ---------------------------------------------------------------------------------------------------
# The TGT radio message (Jetson -> ground, inside a MAVLink STATUSTEXT)
# ---------------------------------------------------------------------------------------------------

TGT_PREFIX = 'TGT'
TGT_CODE_BY_KIND = {KIND_PERSON: 'p', KIND_TENT: 't'}
TGT_KIND_BY_CODE = {code: kind for kind, code in TGT_CODE_BY_KIND.items()}
#: MAVLink cuts a STATUSTEXT after 50 characters, so a TGT message may never be longer.
STATUSTEXT_MAX_CHARS = 50
#: MAVLink severity INFO, the level the Jetson uses for plain information.
STATUSTEXT_SEVERITY_INFO = 6
#: The message counter counts 0..999999 and then starts again at 0 (see report_codec.next_seq).
TGT_SEQ_MAX = 999999

# ---------------------------------------------------------------------------------------------------
# Target clusterer defaults (see clustering.py for the reasoning)
# ---------------------------------------------------------------------------------------------------

#: Two sightings of the same kind closer than this belong to one physical target. 12 m is wide enough for
#: the position error of one photo (GPS up to ~1 s old at ~10 m/s, heading and height errors) and still well
#: below the distance between different objects of one kind in the simulator worlds: the real targets are
#: more than 100 m apart and the two closest decoy people are 36 m apart (test_scenarios.py checks both).
CLUSTER_MERGE_RADIUS_M = 12.0
#: An update is published only when the estimate moved this far from the last published position ...
CLUSTER_UPDATE_MIN_MOVE_M = 1.0
#: ... or the best confidence grew by at least this much (0.01 = one TGT percentage point).
CLUSTER_UPDATE_MIN_CONF_GAIN = 0.02
#: At most this many clusters are kept; a sighting that would start another one is ignored (bounded memory).
CLUSTER_MAX_CLUSTERS = 64
