#!/usr/bin/env python3

"""
Improved Main Controller with Waypoint-Based Target Selection
=============================================================

Architecture:
1. Detection → Aggregation/Selection → Mission Action (clean separation)
2. Waypoint-based scoring (not raw detection-based)
3. Anti-decoy scoring with repeatability rewards
4. Neighbor reinforcement for robustness
5. Commit logic to prevent false positive switching
6. Separate tent and person selectors

Key Improvements:
- Waypoint attribution fixed (capture-time, not publish-time)
- Per-waypoint statistics (hits, max_conf, topk_mean)
- Scoring rule: (topKMean) × (1 + log(1 + hits))
- Neighbor smoothing: smooth[wp] = 1.0·score[wp] + 0.6·score[wp±1]
- Commit conditions: strength, margin, stability, quality gate
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import ImageResult
from mavros_msgs.srv import CommandLong, SetMode, WaypointSetCurrent
from mavros_msgs.msg import WaypointReached, StatusText, WaypointList
from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import String

from wp_sender.parameter import ParameterManager

import time
import math
from collections import defaultdict, deque
from typing import Dict, List, Tuple, Optional
from enum import Enum, auto
import json

# Servo configuration
HUMAN_SERVO_CHANNEL_1 = 9
HUMAN_SERVO_CHANNEL_2 = 10
HUMAN_SERVOS_PWM = 1500

TENT_SERVO_CHANNEL_1 = 11
TENT_SERVO_CHANNEL_2 = 12
TENT_SERVOS_PWM = 1500

# Scoring and commit parameters
TOPK_SIZE = 3  # Use top-3 confidences for mean
NEIGHBOR_WEIGHT = 0.6  # Weight for adjacent waypoints
MIN_SCORE_THRESHOLD = 0.3  # Minimum score to be considered
COMMIT_MARGIN = 0.15  # Winner must beat second place by this margin
STABILITY_COUNT = 2  # Winner must remain stable for N updates
CONFIDENCE_GATE_TENT = 0.60  # Min confidence for tent commit
CONFIDENCE_GATE_PERSON = 0.65  # Min confidence for person commit

# Safety and action parameters
MAX_WAYPOINT_AGE = 30.0  # Reject detections older than 30 seconds
ACTION_TIMEOUT = 10.0  # Timeout for navigation/payload actions
SERVO_DEPLOY_DURATION = 2.0  # How long to hold servo position
MIN_ACTION_INTERVAL = 5.0  # Minimum time between actions
ADJACENT_WAYPOINT_THRESHOLD = 2  # Waypoints within this distance are "adjacent"


class ControllerState(Enum):
    IDLE = auto()  # Waiting for mission start
    COLLECTING_EVIDENCE = auto()  # Receiving detections, not committed yet
    TARGET_COMMITTED = auto()  # Target selected, ready to act
    NAVIGATING_TO_TARGET = auto()  # En route to target waypoint
    AT_TARGET = auto()  # Reached target, ready for payload
    DEPLOYING_PAYLOAD = auto()  # Servo activation in progress
    PAYLOAD_COMPLETE = auto()  # Payload deployed successfully
    RESUMING_MISSION = auto()  # Returning to normal mission flow
    MULTI_TARGET_SEQUENCING = auto()  # Handling multiple targets
    MISSION_COMPLETE = auto()  # All actions done
    ERROR = auto()  # Recoverable error state


class TargetPriority(Enum):
    PERSON_FIRST = auto()  # Person has higher priority
    TENT_FIRST = auto()  # Tent has higher priority
    CLOSEST_FIRST = auto()  # Navigate to nearest target first
    HIGHEST_CONFIDENCE_FIRST = auto()  # Navigate to most confident detection


class WaypointStats:
    """Statistics for a single waypoint and class"""
    def __init__(self):
        self.hits = 0  # Number of times this class was detected at this waypoint
        self.confidences = deque(maxlen=100)  # Bounded buffer - last 100 detections
        self.max_conf = 0.0  # Maximum confidence
        self.topk_mean = 0.0  # Mean of top-K confidences
        self.last_seen_time = None  # Timestamp of last detection
        self.areas = deque(maxlen=100)  # Bounded buffer for areas
    
    def update(self, confidence: float, area: float = 0.0, timestamp: float = None):
        """Add a new detection observation"""
        self.hits += 1
        self.confidences.append(confidence)
        self.max_conf = max(self.max_conf, confidence)
        self.last_seen_time = timestamp or time.time()
        if area > 0:
            self.areas.append(area)
        
        # Compute top-K mean
        sorted_confs = sorted(self.confidences, reverse=True)
        top_k = sorted_confs[:TOPK_SIZE]
        self.topk_mean = sum(top_k) / len(top_k) if top_k else 0.0
    
    def get_score(self) -> float:
        """
        Anti-decoy scoring rule:
        Score = (topKMean) × (1 + log(1 + hits))
        
        Rewards repeatability - decoys typically produce one spike
        """
        if self.hits == 0:
            return 0.0
        return self.topk_mean * (1.0 + math.log(1.0 + self.hits))
    
    def to_dict(self) -> dict:
        """Convert to dictionary for logging"""
        return {
            'hits': self.hits,
            'max_conf': round(self.max_conf, 3),
            'topk_mean': round(self.topk_mean, 3),
            'score': round(self.get_score(), 3)
        }


class TargetSelector:
    """
    Independent selector for one target class (tent or person)
    
    Maintains per-waypoint statistics and applies:
    - Anti-decoy scoring
    - Neighbor reinforcement
    - Commit logic with stability
    """
    def __init__(self, class_name: str, confidence_gate: float, logger):
        self.class_name = class_name
        self.confidence_gate = confidence_gate
        self.logger = logger
        
        # Per-waypoint statistics
        self.waypoint_stats: Dict[int, WaypointStats] = defaultdict(WaypointStats)
        
        # Selection state
        self.best_waypoint = None
        self.best_score = 0.0
        self.committed = False
        self.stability_counter = 0
        self.last_winner = None
        
        # History for debugging
        self.update_count = 0
    
    def add_detection(self, waypoint_id: int, confidence: float, area: float = 0.0, timestamp: float = None):
        """Add a detection observation for a specific waypoint"""
        ts = timestamp if timestamp else time.time()
        self.waypoint_stats[waypoint_id].update(confidence, area, ts)
        self.logger.info(
            f"[{self.class_name}] WP {waypoint_id}: conf={confidence:.3f}, "
            f"hits={self.waypoint_stats[waypoint_id].hits}, "
            f"score={self.waypoint_stats[waypoint_id].get_score():.3f}"
        )
    
    def compute_smoothed_scores(self) -> Dict[int, float]:
        """
        Apply neighbor reinforcement:
        smoothed[wp] = 1.0 × score[wp] + 0.6 × score[wp-1] + 0.6 × score[wp+1]
        """
        raw_scores = {wp: stats.get_score() for wp, stats in self.waypoint_stats.items()}
        smoothed = {}
        
        all_waypoints = sorted(raw_scores.keys())
        for wp in all_waypoints:
            score = raw_scores.get(wp, 0.0)
            prev_score = raw_scores.get(wp - 1, 0.0)
            next_score = raw_scores.get(wp + 1, 0.0)
            smoothed[wp] = 1.0 * score + NEIGHBOR_WEIGHT * prev_score + NEIGHBOR_WEIGHT * next_score
        
        return smoothed
    
    def update_selection(self):
        """
        Update target selection using commit logic:
        1. Strength: smoothed score >= threshold
        2. Margin: winner beats second place by margin
        3. Stability: winner stays winner for N updates
        4. Quality gate: max confidence >= gate threshold
        """
        self.update_count += 1
        
        # Age out stale waypoints (no detections for MAX_WAYPOINT_AGE seconds)
        current_time = time.time()
        stale_waypoints = []
        for wp, stats in self.waypoint_stats.items():
            if stats.last_seen_time and (current_time - stats.last_seen_time) > MAX_WAYPOINT_AGE:
                stale_waypoints.append(wp)
        
        for wp in stale_waypoints:
            self.logger.info(f"[{self.class_name}] Removing stale waypoint {wp}")
            del self.waypoint_stats[wp]
        
        # Get smoothed scores
        smoothed_scores = self.compute_smoothed_scores()
        
        if not smoothed_scores:
            return
        
        # Find top 2 candidates
        sorted_candidates = sorted(smoothed_scores.items(), key=lambda x: x[1], reverse=True)
        
        if len(sorted_candidates) == 0:
            return
        
        first_wp, first_score = sorted_candidates[0]
        second_score = sorted_candidates[1][1] if len(sorted_candidates) > 1 else 0.0
        
        # Check commit conditions
        strength_ok = first_score >= MIN_SCORE_THRESHOLD
        margin_ok = (first_score - second_score) >= COMMIT_MARGIN
        quality_ok = self.waypoint_stats[first_wp].max_conf >= self.confidence_gate
        
        # Check stability
        if first_wp == self.last_winner:
            self.stability_counter += 1
        else:
            self.stability_counter = 1
            self.last_winner = first_wp
        
        stability_ok = self.stability_counter >= STABILITY_COUNT
        
        # Log selection state
        self.logger.info(
            f"[{self.class_name}] Selection: WP {first_wp}, score={first_score:.3f}, "
            f"margin={first_score-second_score:.3f}, stability={self.stability_counter}, "
            f"max_conf={self.waypoint_stats[first_wp].max_conf:.3f}"
        )
        self.logger.info(
            f"[{self.class_name}] Conditions: strength={strength_ok}, margin={margin_ok}, "
            f"quality={quality_ok}, stability={stability_ok}"
        )
        
        # Commit decision
        if strength_ok and margin_ok and quality_ok and stability_ok and not self.committed:
            self.best_waypoint = first_wp
            self.best_score = first_score
            self.committed = True
            self.logger.info(
                f" [{self.class_name}] COMMITTED to waypoint {first_wp} "
                f"(score={first_score:.3f}, conf={self.waypoint_stats[first_wp].max_conf:.3f})"
            )
        elif not self.committed:
            # Update candidate but don't commit yet
            self.best_waypoint = first_wp
            self.best_score = first_score
    
    def get_result(self) -> Tuple[Optional[int], float, bool]:
        """Return (waypoint_id, score, committed)"""
        return (self.best_waypoint, self.best_score, self.committed)
    
    def get_stats_summary(self) -> dict:
        """Get summary for logging/debugging"""
        return {
            'best_waypoint': self.best_waypoint,
            'best_score': round(self.best_score, 3) if self.best_score else 0.0,
            'committed': self.committed,
            'stability': self.stability_counter,
            'waypoint_count': len(self.waypoint_stats),
            'total_updates': self.update_count
        }


class MainControllerAro(Node):
    """
    Improved main controller with waypoint-based target selection
    
    Pipeline:
    1. Receive detections with CAPTURE-TIME waypoint attribution
    2. Update per-waypoint statistics for tent and person
    3. Run independent selectors with anti-decoy scoring
    4. Apply commit logic
    5. Execute mission action (jump to waypoint or add waypoints)
    """
    def __init__(self):
        super().__init__('main_controller_aro')
        
        # Validate configuration parameters
        assert 0 < COMMIT_MARGIN < 1.0, "COMMIT_MARGIN must be in (0, 1)"
        assert TOPK_SIZE >= 1, "TOPK_SIZE must be >= 1"
        assert 0 <= NEIGHBOR_WEIGHT <= 1.0, "NEIGHBOR_WEIGHT must be in [0, 1]"
        assert MIN_SCORE_THRESHOLD > 0, "MIN_SCORE_THRESHOLD must be positive"
        assert CONFIDENCE_GATE_TENT > 0 and CONFIDENCE_GATE_PERSON > 0, "Confidence gates must be positive"
        
        # Subscribers
        self.create_subscription(ImageResult, "/image_detections", self.image_result_cb, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_cb, 1)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.update_waypoint_reached, 1)
        self.create_subscription(ParameterEvent, "/parameter_events", self.parameter_event_cb, 10)
        
        # Publishers
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.target_selection_pub = self.create_publisher(String, '/target_selection', 10)
        
        # Service clients
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_current_client = self.create_client(WaypointSetCurrent, "/mavros/mission/set_current")
        
        # Wait for critical services
        self._wait_for_services()
        
        # Target selectors (one per class)
        self.tent_selector = TargetSelector("TENT", CONFIDENCE_GATE_TENT, self.get_logger())
        self.person_selector = TargetSelector("PERSON", CONFIDENCE_GATE_PERSON, self.get_logger())
        
        # Mission state
        self.waypoints = []
        self.waypoint_reached = 0
        self.param_manager = ParameterManager()
        self.fetch_mission_indices()
        
        # State machine
        self.state = ControllerState.IDLE
        self.previous_state = None
        self.state_entry_time = time.time()
        
        # Target tracking
        self.tent_target_wp = None
        self.person_target_wp = None
        self.tent_action_complete = False
        self.person_action_complete = False
        self.current_target = None  # (waypoint_id, target_type)
        self.pending_targets = []  # Queue for multi-target sequencing
        self.queued_targets = set()  # Track which targets are already queued (prevent duplicates)
        
        # Action state
        self.action_in_progress = False
        self.last_action_time = 0.0
        self.navigation_start_time = None
        self.payload_start_time = None
        self.resume_waypoint = None  # Where to return after target action
        
        # Non-blocking servo deployment state
        self.servo_futures = []  # List of pending servo command futures
        self.servo_deploy_start_time = None
        self.servos_to_deploy = []  # Queue: [(channel, pwm), ...]
        
        # Adjacent target handling
        self.deploy_both_payloads = False
        
        # Navigation progress monitoring
        self._last_nav_check = None  # (timestamp, waypoint)
        
        # Error tracking and statistics
        self.error_count = 0
        self.recovery_count = 0
        self.abort_requested = False  # For operator abort
        
        # Timer for periodic selection updates
        self.create_timer(2.0, self.periodic_selection_update)
        
        # State machine timer
        self.create_timer(0.5, self.state_machine_update)
        
        # Telemetry and debugging timer
        self.create_timer(10.0, self.log_waypoint_stats)
        
        self._transition_state(ControllerState.COLLECTING_EVIDENCE)
        self.get_logger().info(" MainControllerAro initialized with waypoint-based selection")
    
    def _wait_for_services(self):
        """Wait for all required services"""
        services = [
            ("set_mode", self.set_mode_client),
            ("set_current", self.set_current_client),
        ]
        for name, client in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name} service...")
    
    def fetch_mission_indices(self):
        """Fetch mission structure parameters from waypoint manager"""
        wp_params = ['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff', 'last_before_rtl']
        params = self.param_manager.get_param(self.param_manager.waypoint_client, list_params=wp_params)
        
        if params:
            self.num_waypoints = int(params.get('num_waypoints', 0))
            self.takeoff_index = int(params.get('takeoff_index', 0))
            self.rtl_index = int(params.get('rtl_index', 0))
            self.next_after_takeoff = int(params.get('next_after_takeoff', 0))
            self.last_before_rtl = int(params.get('last_before_rtl', 0))
            
            self.get_logger().info(
                f"Mission indices: num_wp={self.num_waypoints}, takeoff={self.takeoff_index}, "
                f"rtl={self.rtl_index}, last_before_rtl={self.last_before_rtl}"
            )
    
    def parameter_event_cb(self, msg: ParameterEvent):
        """Handle parameter updates from waypoint manager"""
        if msg.node == "/waypoint_manager":
            for changed_param in msg.changed_parameters:
                if changed_param.name in {"num_waypoints", "takeoff_index", "rtl_index", 
                                          "next_after_takeoff", "last_before_rtl"}:
                    self.fetch_mission_indices()
                    break
    
    def waypoints_cb(self, msg: WaypointList):
        """Store current mission waypoint list"""
        self.waypoints = msg.waypoints
        self.get_logger().info(f"Mission updated: {len(self.waypoints)} waypoints loaded")
    
    def update_waypoint_reached(self, msg: WaypointReached):
        """Track current waypoint and trigger payload actions at target waypoints"""
        self.waypoint_reached = msg.wp_seq
        self.get_logger().info(f" Reached waypoint {self.waypoint_reached}")
        
        # Check if we reached a target waypoint
        if self.current_target and self.waypoint_reached == self.current_target[0]:
            target_wp, target_type = self.current_target
            self.get_logger().info(f" Arrived at target waypoint {target_wp} ({target_type})")
            
            if self.state == ControllerState.NAVIGATING_TO_TARGET:
                self._transition_state(ControllerState.AT_TARGET)
    
    def image_result_cb(self, msg: ImageResult):
        """
        Process detection results with CAPTURE-TIME waypoint attribution
        
        Critical: msg.waypoint_index should be set at capture time in the detection node,
        NOT at publish time. This ensures correct waypoint attribution even if detection lags.
        """
        if not msg.detections.detections:
            return
        
        # Extract waypoint ID from message (this should be capture-time attribution)
        waypoint_id = msg.waypoint_index
        
        # Validation: reject invalid waypoint indices
        if waypoint_id <= 0:
            self.get_logger().warn(f"Invalid waypoint_index={waypoint_id} in detection message")
            return
        
        # Validation: verify waypoint exists in mission
        if len(self.waypoints) > 0 and waypoint_id >= len(self.waypoints):
            self.get_logger().warn(
                f"Waypoint {waypoint_id} out of range (mission has {len(self.waypoints)} waypoints)"
            )
            return
        
        # Extract timestamp for staleness checking
        detection_timestamp = None
        if hasattr(msg, 'timestamp') and msg.timestamp:
            try:
                if isinstance(msg.timestamp, str):
                    # Try ISO format first
                    try:
                        detection_timestamp = datetime.fromisoformat(msg.timestamp).timestamp()
                    except ValueError:
                        # Try parsing as unix timestamp string
                        try:
                            detection_timestamp = float(msg.timestamp)
                        except ValueError:
                            self.get_logger().debug(f"Could not parse timestamp string: {msg.timestamp}")
                elif isinstance(msg.timestamp, (int, float)):
                    detection_timestamp = float(msg.timestamp)
                elif hasattr(msg.timestamp, 'sec'):  # ROS Time object
                    detection_timestamp = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
                
                if detection_timestamp:
                    age = time.time() - detection_timestamp
                    if age > MAX_WAYPOINT_AGE:
                        self.get_logger().warn(f"Rejecting stale detection (age={age:.1f}s)")
                        return
                    if age < 0:  # Future timestamp - clock skew or bad data
                        self.get_logger().warn(f"Detection has future timestamp (age={age:.1f}s), using current time")
                        detection_timestamp = time.time()
            except Exception as e:
                self.get_logger().debug(f"Could not parse timestamp: {e}")
                # Use current time as fallback
                detection_timestamp = time.time()
        else:
            # No timestamp in message
            detection_timestamp = time.time()
        
        self.get_logger().info(
            f" Processing {len(msg.detections.detections)} detection(s) from waypoint {waypoint_id}"
        )
        
        # Process each detection
        for detection in msg.detections.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                confidence = result.hypothesis.score
                
                # Calculate detection area (optional, for filtering)
                bbox = detection.bbox
                area = bbox.size_x * bbox.size_y if hasattr(bbox, 'size_x') else 0.0
                
                # Route to appropriate selector
                if class_id == "0":  # Person
                    self.person_selector.add_detection(waypoint_id, confidence, area, detection_timestamp)
                elif class_id == "1":  # Tent
                    self.tent_selector.add_detection(waypoint_id, confidence, area, detection_timestamp)
        
        # Trigger selection update after new data
        self.update_target_selection()
    
    def periodic_selection_update(self):
        """Periodically update target selection (even without new detections)"""
        if not self.action_in_progress:
            self.update_target_selection()
    
    def update_target_selection(self):
        """
        Update both selectors and publish results
        Also triggers mission actions if conditions are met
        """
        # Update selectors
        self.tent_selector.update_selection()
        self.person_selector.update_selection()
        
        # Get results
        tent_wp, tent_score, tent_committed = self.tent_selector.get_result()
        person_wp, person_score, person_committed = self.person_selector.get_result()
        
        # Build complete telemetry message
        selection_msg = {
            'tent': {
                'waypoint': tent_wp,
                'score': round(tent_score, 3) if tent_score else 0.0,
                'committed': tent_committed,
                'stats': self.tent_selector.get_stats_summary()
            },
            'person': {
                'waypoint': person_wp,
                'score': round(person_score, 3) if person_score else 0.0,
                'committed': person_committed,
                'stats': self.person_selector.get_stats_summary()
            },
            'controller_state': self.state.name,
            'current_target': {
                'waypoint': self.current_target[0] if self.current_target else None,
                'type': self.current_target[1] if self.current_target else None
            },
            'pending_targets': len(self.pending_targets),
            'action_in_progress': self.action_in_progress,
            'timestamp': time.time()
        }
        
        # Publish complete message
        msg = String()
        msg.data = json.dumps(selection_msg)
        self.target_selection_pub.publish(msg)
        
        # Log current state
        self.get_logger().info(
            f" Selection: TENT[wp={tent_wp}, score={tent_score:.2f}, committed={tent_committed}] "
            f"PERSON[wp={person_wp}, score={person_score:.2f}, committed={person_committed}] State={self.state.name}"
        )
        
        # Update target tracking when committed
        if tent_committed and self.tent_target_wp != tent_wp:
            self.tent_target_wp = tent_wp
            self.get_logger().info(f" TENT target locked: waypoint {tent_wp}")
        
        if person_committed and self.person_target_wp != person_wp:
            self.person_target_wp = person_wp
            self.get_logger().info(f" PERSON target locked: waypoint {person_wp}")
        
        # Check if we should transition to TARGET_COMMITTED state
        if (tent_committed or person_committed) and self.state == ControllerState.COLLECTING_EVIDENCE:
            self._transition_state(ControllerState.TARGET_COMMITTED)
    
    def _coordinate_multi_target(self) -> Optional[Tuple[int, str]]:
        """
        Coordinate multiple targets with priority logic
        
        Returns: (waypoint_id, target_type) to navigate to, or None
        """
        tent_ready = self.tent_target_wp and not self.tent_action_complete
        person_ready = self.person_target_wp and not self.person_action_complete
        
        if not tent_ready and not person_ready:
            return None
        
        # Only one target ready
        if tent_ready and not person_ready:
            return (self.tent_target_wp, "tent")
        if person_ready and not tent_ready:
            return (self.person_target_wp, "person")
        
        # Both targets ready - apply coordination logic
        tent_wp = self.tent_target_wp
        person_wp = self.person_target_wp
        
        # Case 1: Same or adjacent waypoints - go to higher confidence first, then deploy both
        if abs(tent_wp - person_wp) <= ADJACENT_WAYPOINT_THRESHOLD:
            tent_conf = self.tent_selector.waypoint_stats[tent_wp].max_conf
            person_conf = self.person_selector.waypoint_stats[person_wp].max_conf
            
            # Choose which waypoint to navigate to (higher confidence)
            if person_conf > tent_conf:
                primary_target = (person_wp, "person")
                self.get_logger().info(
                    f" Adjacent targets at wp {person_wp}/{tent_wp}: Will deploy both payloads"
                )
            else:
                primary_target = (tent_wp, "tent")
                self.get_logger().info(
                    f" Adjacent targets at wp {tent_wp}/{person_wp}: Will deploy both payloads"
                )
            
            # Flag to deploy both payloads at this location
            self.deploy_both_payloads = True
            return primary_target
        
        # Case 2: Far apart - use priority system (PERSON_FIRST by default)
        priority = TargetPriority.PERSON_FIRST
        
        if priority == TargetPriority.PERSON_FIRST:
            self.get_logger().info(f" Multi-target: Person first (wp={person_wp}), then Tent (wp={tent_wp})")
            # Only queue if not already queued
            tent_key = (tent_wp, "tent")
            if tent_key not in self.queued_targets:
                self.pending_targets.append(tent_key)
                self.queued_targets.add(tent_key)
            return (person_wp, "person")
        elif priority == TargetPriority.CLOSEST_FIRST:
            current_wp = self.waypoint_reached
            dist_tent = abs(tent_wp - current_wp)
            dist_person = abs(person_wp - current_wp)
            
            if dist_person < dist_tent:
                tent_key = (tent_wp, "tent")
                if tent_key not in self.queued_targets:
                    self.pending_targets.append(tent_key)
                    self.queued_targets.add(tent_key)
                return (person_wp, "person")
            else:
                person_key = (person_wp, "person")
                if person_key not in self.queued_targets:
                    self.pending_targets.append(person_key)
                    self.queued_targets.add(person_key)
                return (tent_wp, "tent")
        
        # Default: person first
        tent_key = (tent_wp, "tent")
        if tent_key not in self.queued_targets:
            self.pending_targets.append(tent_key)
            self.queued_targets.add(tent_key)
        return (person_wp, "person")
    
    def _navigate_to_target(self, waypoint_id: int, target_type: str) -> bool:
        """
        Command vehicle to go to specific waypoint using mission set_current
        
        Returns: True if navigation command succeeded, False otherwise
        """
        # Safety: rate limiting
        if time.time() - self.last_action_time < MIN_ACTION_INTERVAL:
            self.get_logger().warn("Action rate limit - too soon since last action")
            return False
        
        # Safety: validate waypoint
        if len(self.waypoints) > 0 and waypoint_id >= len(self.waypoints):
            self.get_logger().error(f"Cannot navigate to invalid waypoint {waypoint_id}")
            return False
        
        try:
            # Store resume point (current waypoint before jump)
            self.resume_waypoint = self.waypoint_reached
            
            req = WaypointSetCurrent.Request()
            req.wp_seq = waypoint_id
            
            future = self.set_current_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f" Navigation started: waypoint {waypoint_id} ({target_type})")
                self.send_ack(f"Navigating to {target_type} at waypoint {waypoint_id}")
                self.current_target = (waypoint_id, target_type)
                self.navigation_start_time = time.time()
                self.last_action_time = time.time()
                self._last_nav_check = None  # Reset navigation progress tracking
                return True
            else:
                self.get_logger().error(f" Failed to set mission waypoint to {waypoint_id}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Navigation command failed: {e}")
            return False
    
    def change_mode(self, mode: str):
        """Change vehicle flight mode"""
        try:
            req = SetMode.Request()
            req.custom_mode = mode
            
            future = self.set_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            response = future.result()
            if response and response.mode_sent:
                self.get_logger().info(f" Mode changed to {mode}")
            else:
                self.get_logger().error(f" Failed to change mode to {mode}")
        
        except Exception as e:
            self.get_logger().error(f"Error changing mode: {e}")
    
    def move_servo(self, channel: int, pwm: int):
        """Send servo command to vehicle"""
        try:
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = float(channel)
            request.param2 = float(pwm)
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0
            
            future = self.command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            response = future.result()
            if response and response.success:
                self.get_logger().info(f" Servo channel {channel} → {pwm}μs")
            else:
                self.get_logger().warn(f" Failed to move servo channel {channel}")
        
        except Exception as e:
            self.get_logger().error(f"Servo command failed: {e}")
    
    def _deploy_payload_start(self, target_type: str):
        """
        Start non-blocking payload deployment (queue servos)
        """
        self.servo_deploy_start_time = time.time()
        self.servo_futures.clear()
        
        # Check if we need to deploy both payloads (adjacent targets)
        if self.deploy_both_payloads:
            # Verify both targets exist and are valid
            if not self.tent_target_wp or not self.person_target_wp:
                self.get_logger().error(" Both payloads flag set but missing target waypoint!")
                self.deploy_both_payloads = False
                # Fall through to single-target logic
            else:
                self.get_logger().info(" Deploying both person AND tent payloads (adjacent targets)")
                self.send_ack("Deploying both markers")
                self.servos_to_deploy = [
                    (HUMAN_SERVO_CHANNEL_1, HUMAN_SERVOS_PWM),
                    (HUMAN_SERVO_CHANNEL_2, HUMAN_SERVOS_PWM),
                    (TENT_SERVO_CHANNEL_1, TENT_SERVOS_PWM),
                    (TENT_SERVO_CHANNEL_2, TENT_SERVOS_PWM)
                ]
                # Mark both as complete
                self.tent_action_complete = True
                self.person_action_complete = True
                self.deploy_both_payloads = False  # Reset flag
                return  # Exit early - don't process single target logic
        
        # Single target deployment
        if target_type == "person":
            self.get_logger().info(" Deploying person payload")
            self.send_ack("Deploying person marker")
            self.servos_to_deploy = [
                (HUMAN_SERVO_CHANNEL_1, HUMAN_SERVOS_PWM),
                (HUMAN_SERVO_CHANNEL_2, HUMAN_SERVOS_PWM)
            ]
        elif target_type == "tent":
            self.get_logger().info(" Deploying tent payload")
            self.send_ack("Deploying tent marker")
            self.servos_to_deploy = [
                (TENT_SERVO_CHANNEL_1, TENT_SERVOS_PWM),
                (TENT_SERVO_CHANNEL_2, TENT_SERVOS_PWM)
            ]
        else:
            self.servos_to_deploy = []
    
    def _deploy_payload_update(self) -> Optional[bool]:
        """
        Non-blocking payload deployment update (call from state machine)
        
        Returns: None = in progress, True = success, False = failed
        """
        if not self.servos_to_deploy and not self.servo_futures:
            return True  # Nothing to do
        
        # Check if we need to send next servo command
        if self.servos_to_deploy and len(self.servo_futures) == 0:
            # Send first/next servo command
            channel, pwm = self.servos_to_deploy.pop(0)
            result = self._move_servo_async(channel, pwm)
            if result:
                self.servo_futures.append(result)
            else:
                self.get_logger().error(f"Failed to send servo command for channel {channel} after retries")
                return False
        
        # Check pending futures
        completed_futures = []
        for i, (future, channel, start_time, retry_count) in enumerate(self.servo_futures):
            if future.done():
                try:
                    response = future.result()
                    if response and response.success:
                        self.get_logger().info(f" Servo {channel} completed")
                        completed_futures.append(i)
                    else:
                        self.get_logger().warn(f" Servo {channel} failed")
                        return False  # Deployment failed
                except Exception as e:
                    self.get_logger().error(f"Servo {channel} exception: {e}")
                    return False
            elif time.time() - start_time > 3.0:
                # Timeout
                self.get_logger().warn(f"Servo {channel} timeout")
                return False
        
        # Remove completed futures
        for i in reversed(completed_futures):
            self.servo_futures.pop(i)
        
        # If a servo just completed and we have more to deploy, wait briefly before next
        if completed_futures and self.servos_to_deploy:
            # Check if enough time has passed since last completion
            if self.servo_deploy_start_time and time.time() - self.servo_deploy_start_time < 0.5:
                return None  # Still waiting for delay
            # Reset timer for next servo
            self.servo_deploy_start_time = time.time()
        
        # Check if all done
        if not self.servos_to_deploy and not self.servo_futures:
            self.get_logger().info(" All servos deployed successfully")
            return True
        
        return None  # Still in progress
    
    def _move_servo_async(self, channel: int, pwm: int, retry_count: int = 0):
        """
        Async servo move with retry logic
        
        Returns: (future, channel, start_time, retry_count) tuple or None on fatal error
        """
        try:
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = float(channel)
            request.param2 = float(pwm)
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0
            
            future = self.command_client.call_async(request)
            if retry_count > 0:
                self.get_logger().info(f" Servo {channel} → {pwm}μs (retry {retry_count})")
            else:
                self.get_logger().info(f" Servo {channel} → {pwm}μs (async)")
            return (future, channel, time.time(), retry_count)
        
        except Exception as e:
            if retry_count < 3:
                self.get_logger().warn(f"Servo {channel} failed, retry {retry_count + 1}/3")
                # Don't sleep - let state machine handle retry timing
                return self._move_servo_async(channel, pwm, retry_count + 1)
            else:
                self.get_logger().error(f"Servo {channel} failed after 3 retries: {e}")
                return None
    
    def _resume_normal_mission(self) -> bool:
        """
        Return to normal mission flow after target action
        
        Returns: True if resume succeeded
        """
        if self.resume_waypoint is None:
            # If no resume point, check if there are pending targets
            if self.pending_targets:
                next_target = self.pending_targets.pop(0)
                self.queued_targets.discard(next_target)  # Remove from queued set
                self.get_logger().info(f" Proceeding to next target: {next_target}")
                return self._navigate_to_target(next_target[0], next_target[1])
            else:
                # All actions complete - continue mission
                self.get_logger().info(" All target actions complete - continuing mission")
                self.send_ack("Target objectives complete")
                self._transition_state(ControllerState.MISSION_COMPLETE)
                return True
        
        # Resume at next waypoint after the resume point
        next_wp = self.resume_waypoint + 1
        
        # Check if we should go to next pending target instead
        if self.pending_targets:
            next_target = self.pending_targets.pop(0)
            self.queued_targets.discard(next_target)  # Remove from queued set
            self.get_logger().info(f" Next target in queue: {next_target}")
            return self._navigate_to_target(next_target[0], next_target[1])
        
        # Resume normal mission
        if next_wp < len(self.waypoints):
            self.get_logger().info(f" Resuming mission at waypoint {next_wp}")
            self.send_ack(f"Resuming mission at waypoint {next_wp}")
            
            try:
                req = WaypointSetCurrent.Request()
                req.wp_seq = next_wp
                future = self.set_current_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    self.resume_waypoint = None
                    self._transition_state(ControllerState.MISSION_COMPLETE)
                    return True
                else:
                    self.get_logger().error("Failed to resume mission")
                    return False
            except Exception as e:
                self.get_logger().error(f"Resume mission error: {e}")
                return False
        else:
            self.get_logger().info(" Mission complete")
            self._transition_state(ControllerState.MISSION_COMPLETE)
            return True
    
    def _check_navigation_progress(self) -> bool:
        """
        Check if navigation is making progress toward target
        
        Returns: True if making progress, False if stuck
        """
        if not self.current_target:
            return True
        
        target_wp = self.current_target[0]
        current_wp = self.waypoint_reached
        
        # If we're at or past target, we're done (AT_TARGET will trigger)
        if current_wp >= target_wp:
            return True
        
        # Check if we're advancing through waypoints
        if self._last_nav_check is None:
            self._last_nav_check = (time.time(), current_wp)
            return True
        
        last_time, last_wp = self._last_nav_check
        time_elapsed = time.time() - last_time
        
        # Check every 15 seconds (allow time for slower missions)
        if time_elapsed > 15.0:
            if current_wp == last_wp:
                # No waypoint progress in 15 seconds - might be stuck
                self.get_logger().warn(
                    f" Navigation progress check: at wp {current_wp}, target {target_wp}, "
                    f"no advancement in {time_elapsed:.1f}s"
                )
                # Only fail if we haven't started moving at all from resume point
                if hasattr(self, 'resume_waypoint') and self.resume_waypoint and current_wp == self.resume_waypoint:
                    self.get_logger().error("Navigation completely stuck at resume point")
                    return False
            # Update checkpoint
            self._last_nav_check = (time.time(), current_wp)
        
        return True
    
    def _transition_state(self, new_state: ControllerState):
        """Transition to a new state with logging"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_entry_time = time.time()
            self.get_logger().info(f" State: {self.previous_state.name} → {new_state.name}")
            self.send_ack(f"State: {new_state.name}")
    
    def _should_abort(self) -> bool:
        """
        Check if mission should be aborted
        
        Returns: True if abort conditions are met
        """
        # Check operator abort flag
        if self.abort_requested:
            return True
        
        # Check for too many errors
        if self.error_count > 5:
            self.get_logger().error(f"Too many errors ({self.error_count}), aborting")
            return True
        
        # Add other abort conditions here (e.g., battery low, geofence)
        
        return False
    
    def state_machine_update(self):
        """Main state machine - runs every 0.5 seconds"""
        
        # Check for abort conditions first (before any state processing)
        if self._should_abort():
            self.abort_mission("Operator abort or critical error")
            return
        
        time_in_state = time.time() - self.state_entry_time
        
        if self.state == ControllerState.IDLE:
            # Waiting for mission to start
            if len(self.waypoints) > 0:
                self._transition_state(ControllerState.COLLECTING_EVIDENCE)
        
        elif self.state == ControllerState.COLLECTING_EVIDENCE:
            # Continuously collecting detections - selection update handles transition
            pass
        
        elif self.state == ControllerState.TARGET_COMMITTED:
            # Ready to navigate to target
            if not self.action_in_progress:
                target = self._coordinate_multi_target()
                if target:
                    # Only set flag when we actually have a target and are starting navigation
                    self.action_in_progress = True
                    waypoint_id, target_type = target
                    if self._navigate_to_target(waypoint_id, target_type):
                        self._transition_state(ControllerState.NAVIGATING_TO_TARGET)
                    else:
                        self.get_logger().error("Navigation command failed")
                        self.action_in_progress = False
                        self._transition_state(ControllerState.ERROR)
                # else: No targets ready yet, stay in TARGET_COMMITTED and try again next tick
        
        elif self.state == ControllerState.NAVIGATING_TO_TARGET:
            # Check navigation progress
            if not self._check_navigation_progress():
                self.abort_mission("Navigation stuck - no progress")
            
            # Wait for waypoint_reached callback to trigger AT_TARGET transition
            # Extended timeout for genuine issues
            if self.navigation_start_time and time_in_state > ACTION_TIMEOUT * 3:
                self.abort_mission(f"Navigation timeout after {time_in_state:.1f}s")
        
        elif self.state == ControllerState.AT_TARGET:
            # Ready to deploy payload - start non-blocking deployment
            if self.current_target:
                _, target_type = self.current_target
                self._deploy_payload_start(target_type)
                self._transition_state(ControllerState.DEPLOYING_PAYLOAD)
        
        elif self.state == ControllerState.DEPLOYING_PAYLOAD:
            # Non-blocking payload deployment update
            result = self._deploy_payload_update()
            if result is True:
                # Deployment succeeded
                self.get_logger().info(" Payload deployment complete")
                self._transition_state(ControllerState.PAYLOAD_COMPLETE)
            elif result is False:
                # Deployment failed
                self.get_logger().error(" Payload deployment failed")
                # Continue anyway to avoid getting stuck
                self._transition_state(ControllerState.PAYLOAD_COMPLETE)
            # else: result is None, still in progress - wait for next tick
        
        elif self.state == ControllerState.PAYLOAD_COMPLETE:
            # Mark target as complete and clean up queued set
            if self.current_target:
                _, target_type = self.current_target
                if target_type == "person":
                    self.person_action_complete = True
                    # Clean up queued set
                    if self.person_target_wp:
                        self.queued_targets.discard((self.person_target_wp, "person"))
                elif target_type == "tent":
                    self.tent_action_complete = True
                    # Clean up queued set
                    if self.tent_target_wp:
                        self.queued_targets.discard((self.tent_target_wp, "tent"))
            else:
                # Edge case: no current target but in PAYLOAD_COMPLETE?
                self.get_logger().warn(" In PAYLOAD_COMPLETE but no current_target - possible state corruption")
            
            self.current_target = None
            self._transition_state(ControllerState.RESUMING_MISSION)
        
        elif self.state == ControllerState.RESUMING_MISSION:
            # Resume normal mission or go to next target
            if self._resume_normal_mission():
                # State transition handled in resume function
                self.action_in_progress = False  # Clear action flag when done
            else:
                self.get_logger().error("Failed to resume mission")
                self.action_in_progress = False
                self._transition_state(ControllerState.ERROR)
        
        elif self.state == ControllerState.MISSION_COMPLETE:
            # All done - clear action flag and stay here
            self.action_in_progress = False
        
        elif self.state == ControllerState.ERROR:
            # Recoverable error - try to resume after timeout
            if time_in_state > 10.0:
                self.recovery_count += 1
                self.get_logger().info(
                    f"Attempting recovery from error state "
                    f"(errors: {self.error_count}, recoveries: {self.recovery_count})"
                )
                self.action_in_progress = False
                self._transition_state(ControllerState.COLLECTING_EVIDENCE)
    
    def abort_mission(self, reason: str):
        """Emergency abort - RTL and cleanup"""
        self.error_count += 1
        self.get_logger().error(f" MISSION ABORT #{self.error_count}: {reason}")
        self.send_ack(f"ABORT: {reason}")
        
        # Switch to RTL mode
        self.change_mode("RTL")
        
        # Clear all targets and state
        self.action_in_progress = False
        self.pending_targets.clear()
        self.queued_targets.clear()
        self.current_target = None
        self.deploy_both_payloads = False
        
        self._transition_state(ControllerState.ERROR)
    
    def log_waypoint_stats(self):
        """Periodically log full waypoint statistics for debugging"""
        try:
            tent_stats = {wp: stats.to_dict() 
                          for wp, stats in self.tent_selector.waypoint_stats.items()}
            person_stats = {wp: stats.to_dict() 
                            for wp, stats in self.person_selector.waypoint_stats.items()}
            
            if tent_stats:
                self.get_logger().info(f" TENT waypoint stats: {json.dumps(tent_stats)}")
            if person_stats:
                self.get_logger().info(f" PERSON waypoint stats: {json.dumps(person_stats)}")
            
            # Log current mission state
            self.get_logger().info(
                f" State: {self.state.name}, Action: {self.action_in_progress}, "
                f"Tent: {self.tent_action_complete}, Person: {self.person_action_complete}, "
                f"Queue: {len(self.pending_targets)}, Errors: {self.error_count}, Recoveries: {self.recovery_count}"
            )
        except Exception as e:
            self.get_logger().debug(f"Error logging stats: {e}")
    
    def send_ack(self, text: str):
        """Send status message to ground station"""
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f" Status: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = MainControllerAro()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
