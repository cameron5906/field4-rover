# FIELD4 v0.5: historical planning and discrete-action contracts

**Historical, superseded reference.** This privacy-edited edition preserves a source audit dated 11 October 2026 for the experimental FIELD4 v0.5 implementation, associated with Godot 4.6.3. It is not documentation of the current public v07.3 source subset, a runnable build, or a validated operating procedure.

All implementation details, constants, timings, schemas and defects below describe that old audit. They have not been revalidated against the old executable or a live service. In particular, every `file:line` reference belongs to the historical v0.5 code and **does not identify corresponding lines or necessarily equivalent behavior in the public v07.3 subset**. Several referenced files, including the native transport and application entry point, are intentionally absent from that public subset.

Transport descriptions are provider-neutral. Model identifiers are replaced with an explicit placeholder, and the original endpoint names are omitted. Retained request/response shapes and instruction text document historical application expectations; they are **not verified current or public API documentation**. The request examples are explanatory and must not be copied as supported service integrations. Owner-specific spending limits, reservations, pricing formulas and account/credential context are excluded.

No live API call or hardware test was made for the original audit or this editorial edition. Hardware commissioning, sustained following, and end-to-end bedroom-mission success are not established. Historical simulator velocity/guard values must not be used as validated hardware safety limits.

## 1. Architecture and cadence

The native path has three distinct layers:

1. `semantic_planner.gd` constructs candidate viewpoints from measured occupancy, asks a configured semantic model for a semantic subgoal, maintains short session memory and derives a local waypoint.
2. `native_decisions_client.gd` asks a configured model to choose one bounded motion and answer a hazard predicate. It does not ask for continuous motor values.
3. `decisions_adapter.gd` and `main.gd` validate the response and impose deterministic motion, timing, lidar, collision and tilt limits.

The audited application used separate HTTPS request paths for semantic planning and discrete motion selection. The provider-specific model identifier and endpoint names are intentionally not reproduced. Neither historical payload contained a system or developer message. The planner instruction was text prepended to a single **user** message; discrete-action instructions were placed in three question objects. “Decisions” below is the old application/module label, not a claim that a service with that name is publicly available.

### Motion loop

`main.gd:932–985` and `652–731`: each physics update attempts to queue a capture if AI is on and neither a capture nor a Decisions request is pending. Capture waits for `RenderingServer.frame_post_draw`, takes a new camera/map image and reads current measured state. There is no FIFO of old observations and no fixed two-Hz request timer. After a response is handled, the next frame may start the next request, even before the prior 500 ms action has expired. Thus the rate is response/render/sensor dependent; one new response can replace a still-running command. If inference takes longer than the bounded action, the watchdog stops the robot while it waits.

There is at most one in-flight Decisions request and one in-flight planner request. Those two can overlap: while replanning, an existing semantic subgoal can still supply motion context. With no subgoal, the app stops and waits for the planner. Physics defaults to 120 Hz in rigid-body mode; lidar is sampled every 0.2 simulation seconds. A capture requires a scan no older than 0.45 simulation seconds.

### Planner loop

`semantic_planner.gd:6–8,80–107,152–164`: planner observations/candidate rebuilds are throttled to once per 1,000 ms. Planning is no more frequent than once per 6,000 ms, with periodic reassessment at 25,000 ms. Triggers include mission change, resume, goal unreachable, goal reached within 0.30 m, more than 180 newly observed cells, changed route, failure retry, or apparent lack of progress. The stuck heuristic examines the last three recent actions: at least two must have measured outcomes, and none may exceed 0.04 m translation or 0.10 rad absolute yaw change. Pending requests, `found` status, or three accumulated failed planner attempts prevent a new request. AI restart/resume resets the failure count. The network deadline is 20 seconds; the semantic acceptance layer separately allows at most 25 seconds from capture.

A failed replan can leave the old subgoal in place; failure does not automatically erase it. A `found` or `blocked` plan stops and cancels AI. Bridge mode (`--enable-bridge`) bypasses this native semantic-planning integration and uses localhost Decisions transport; it is a separate execution path.

## 2. Shared observation envelope and model-visible input

`decisions_adapter.gd:96–112` creates this local application envelope (not the exact provider request):

```json
{
  "session_id": "field4-...",
  "observation_seq": 42,
  "captured_at_ms": 123456,
  "mission_version": 1,
  "camera_image": "data:image/jpeg;base64,...",
  "map_image": "data:image/png;base64,...",
  "state": {}
}
```

`captured_at_ms` uses Godot's monotonic ticks, not Unix time. Both the planner and Decisions receive actual vision inputs. With the current default camera profile, the robot POV renders at 640 × 360 and is encoded at 480 × 270; the 200 × 160 occupancy-map PNG remains 200 × 160. Both are sent with `detail: "low"`. The planner builds its content by deep-copying these exact image parts from the Decisions payload (`native_decisions_client.gd:443–449`). Camera JPEG quality is 0.75. Each image is resized proportionally only if its largest dimension exceeds 480. The native validator allows PNG/JPEG data URLs, at most 900,000 characters each and decoded dimensions at most 512 × 512. `session_id` must be a nonempty string of at most 100 characters; sequence/capture time must be nonnegative integers. The state must be a nonempty dictionary, its projected floats finite, and its serialized projection no longer than 20,000 characters. Request bodies are capped at 2,000,000 bytes. Request start rejects future observations or ones over 2,000 ms old.

The API receives the following content array: input text, camera image (`detail: "low"`), occupancy-map image (`detail: "low"`). It does **not** receive the entire internal observation dictionary. `native_decisions_client.gd:265–297` explicitly projects these state keys:

```text
observation_quality, clearances, map_progress, controller_context,
recent_actions, odometry, lidar_ranges_m, front_clearance_m,
rear_clearance_m, rear_scan_valid, lidar_frame, velocity_m_s,
angular_velocity_rad_s, task, mission_version, map_legend, planner_context
```

`imu` is separately projected to these allowed fields, when present:

```text
model, provenance, calibrated, valid, sequence, timestamp_s, age_s, stale,
sample_interval_s, accel_saturated, gyro_saturated, axes, accel_m_s2,
gyro_rad_s, magnetometer_raw_counts, magnetic_heading_untrusted,
magnetic_disturbance_active
```

No house-layout truth, scene object identities, absolute ground-truth pose, hidden room labels or collision-world geometry are sent in this projection. The local simulator still uses world physics for collision supervision.

### State definitions and units

`main.gd:571–598,770–774`, `mapping.gd:3–28`:

- `task`: mission string. Default: `Explore the house and expand the observed map safely.` Mission input is trimmed, nonempty and at most 500 original characters.
- `odometry`: `{ "x": number, "z": number, "yaw": number }`. Positions are metres in a local estimated odometry frame; yaw is radians. A Godot Vector3 pose internally means `(X,Z,yaw)`, not spatial `(X,Y,Z)`.
- `velocity_m_s` / `angular_velocity_rad_s`: encoder-derived linear and yaw velocity. Positive translation is robot forward, local −Z; positive yaw turns the nose left, toward local −X.
- `lidar_ranges_m`: exactly 180 finite numbers. `-1` is invalid/unknown; otherwise values must lie in `[0.15,6.0]`. Ray angle is `i*2π/180`: index 0 forward, 45 left, 90 rear, 135 right. Max-range returns are 6 m. The lidar's simulated origin is `(0,+0.062)` m in robot XZ, with assumed height 0.164 m.
- `clearances`: `front`, `left`, `rear`, `right`, each `{ "valid": boolean, "distance_m": number }`. Each is the minimum across ±10 rays (±20°), and invalid rays invalidate that summary. `front_clearance_m`, `rear_clearance_m`, `rear_scan_valid` mirror these directional summaries. They are not the full swept-body safety check.
- `observation_quality`: `sequence`, `captured_monotonic_ms`, `lidar_age_s`, `lidar_valid_rays`, `camera`, `localization`. Camera is robot POV with no rear camera. Localization is encoder/IMU/local scan matching, with drift and no loop closure.
- `map_progress`: `observed_cells`, `free_cells`, `occupied_cells`, `grid_cells`, `observed_fraction_of_fixed_grid`, `grid_cell_m`, `robot_column`, `robot_row`, `robot_yaw_rad`, `encoder_distance_m`, `stationary_seconds`, `meaning`. Coverage is observed fraction of a fixed grid, **not** completed-house fraction.
- `map_legend` is a JSON-encoded string, not a nested object. Its contents include `resolution_m`, `origin_xz`, `width`, `height`, `legend`. The historical main instantiates an initially empty 200 × 160 grid at 0.1 m resolution, origin `(-9,-9)` m; this is a generic coordinate window, not a stored home layout. Image columns increase +X and rows +Z; gray is unknown, white free, dark occupied. There is no robot icon; grid coordinates locate the robot in the scaled image.
- `controller_context`: `last_selected`, `last_applied`, `last_model_reason`, `last_guard`, `duration_s` (0.5), `max_linear_m_s` (0.22), `max_yaw_rad_s` (0.65), `reverse_clear`, `reverse_left_clear`, `reverse_right_clear`, `note`. Reverse flags are measured swept-footprint checks, not recommended actions.
- `recent_actions`: last six records with `mission_version`, `sequence`, `selected`, `applied`, `model_reason`, `guard`; measured subsequent outcomes may add `observed_translation_m`, `observed_yaw_delta_rad`, `new_observed_cells`. Outcomes use estimated pose/map change, not body truth.

## 3. Planner request, schema and local planning logic

`native_decisions_client.gd:442–451` wraps the same two images and projected state as:

```json
{
  "model": "<historical-model-id>",
  "service_tier": "default",
  "store": false,
  "input": [{ "role": "user", "content": ["input_text and two input_image objects"] }],
  "reasoning": { "mode": "standard", "effort": "high" },
  "max_output_tokens": 8192,
  "text": {
    "format": {
      "type": "json_schema",
      "name": "robot_semantic_plan",
      "strict": true,
      "schema": "the exact object below"
    }
  }
}
```

The strings used above for `content` and `schema` are explanatory placeholders, not wire values. Exact planner output schema (`semantic_planner.gd:166–167`):

```json
{
  "type": "object",
  "additionalProperties": false,
  "required": ["status", "goal_id", "room_type", "confidence", "evidence", "summary"],
  "properties": {
    "status": { "type": "string", "enum": ["search", "inspect", "found", "blocked"] },
    "goal_id": { "type": "string", "maxLength": 64 },
    "room_type": { "type": "string", "enum": ["bedroom", "living_room", "kitchen", "bathroom", "hallway", "unknown"] },
    "confidence": { "type": "number", "minimum": 0, "maximum": 1 },
    "evidence": { "type": "array", "maxItems": 7, "items": { "type": "string", "enum": ["bed", "sofa", "stove", "sink", "toilet", "door", "no_identifying_feature"] } },
    "summary": { "type": "string", "maxLength": 180 }
  }
}
```

The schema does not enumerate dynamic goal IDs; application validation does. `goal_id` must equal `look_here` or one of the candidates supplied with that exact request, including for `found`/`blocked` outputs. The semantic validator also rejects duplicate evidence strings, although the JSON schema does not contain `uniqueItems`.

For a planning request, `planner_context` is replaced with:

```text
mission: string
mission_version: integer
trigger: string
current_plan: subgoal object, possibly empty
room_hypotheses: array, at most 12
visited_odometry_positions: array of {x,z}, at most 60
candidates: array, at most 12
observation_seq: integer
policy: instruction string
```

Each candidate contains exact fields `id`, `x`, `z`, `kind`, `cell_x`, `cell_y`; its ID is `cell_<column>_<row>`. `kind` is `frontier_viewpoint` or `observed_free_viewpoint`. Coordinates are metres rounded to 0.01.

### Where candidates and routes come from

`semantic_planner.gd:67–150,252–273`:

- A cell is free only when observed and its occupancy log odds are at most −0.2. A candidate/path cell is safe only if all cells within a 0.20 m inflated radius are observed-free.
- Four-connected BFS starts at the current estimated cell and traverses only safe cells. This builds a parent tree of reachable space.
- Candidate sampling considers cells whose column and row are both divisible by four, at least 0.50 m from the robot. Unknown-neighbor score samples six offsets: `(±5,0)`, `(0,±5)`, `(4,4)`, `(-4,-4)` cells.
- Score = `3 * sampled_unknown_count - 0.25 * distance_m - visit_penalty`, with penalty 2 for every saved visit within 0.65 m. Greedy descending-score selection spaces candidates by at least 0.75 m and caps them at 12.
- The model chooses an ID; it cannot generate arbitrary target coordinates. Acceptance rebuilds reachability from the current map and rechecks the chosen candidate's cell.
- For Decisions, `planner_context` instead contains `mission`, `planner_status`, `subgoal`, `plan_revision`, `pending_replan`, last four `room_hypotheses`, `allow_mission_complete`, `navigation_valid`, `instruction`, and optionally `waypoint`.
- `subgoal` contains `goal_id`, `status`, `summary`, `room_type`, `visual_evidence`, `confidence`, and `x`,`z` when a navigable goal exists. The parent tree provides a route; the next waypoint is approximately 0.40 m along it (or the endpoint if nearer).
- `waypoint` fields: `x`, `z`, `distance_m`, `heading_error_rad`, `positive_heading: "turn_left"`. Heading error is `wrap(atan2(-delta_x,-delta_z)-yaw,-π,π)`.
- Without a valid route, translation is disallowed locally; safe rotation may inspect the current view. The model receives a waypoint instruction, not a full optimal route or global room map.

### What `found` actually means

`semantic_planner.gd:220–249` detects target room by a case-insensitive substring search in this fixed order: `bedroom`, `living room`, `kitchen`, `bathroom`. A `found` response is accepted only when room type matches that target, confidence is at least 0.8, evidence includes the corresponding identifying token (`bed`, `sofa`, `stove`, `toilet`), and current estimated pose is less than 0.25 m and 0.35 rad from the captured pose. It is a model visual claim subject to deterministic consistency/freshness checks, **not independent object detection or ground-truth proof**. No `found` completion exists for a generic exploration task or hallway-only task under this validator. A reason answer of `mission_complete` in Decisions alone does not mark the mission found.

## 4. Decisions request and response contract

`native_decisions_client.gd:291–297` sends:

```json
{
  "model": "<historical-model-id>",
  "input": [{ "role": "user", "content": ["input_text and two input_image objects"] }],
  "questions": [
    {
      "type": "choice", "name": "motion", "instructions": "MOTION_INSTRUCTIONS below",
      "choices": [{"value":"stop"},{"value":"forward"},{"value":"reverse"},{"value":"reverse_left"},{"value":"reverse_right"},{"value":"turn_left"},{"value":"turn_right"}]
    },
    {
      "type": "predicate", "name": "hazard",
      "instructions": "Is the robot visibly colliding, unstable, or is sensor evidence too corrupted to choose any safe bounded motion?"
    },
    {
      "type": "choice", "name": "reason", "instructions": "REASON_INSTRUCTIONS below",
      "choices": [{"value":"progress"},{"value":"reposition"},{"value":"obstacle"},{"value":"insufficient_evidence"},{"value":"mission_complete"}]
    }
  ]
}
```

Again, explanatory strings for `content` and the two instruction constants stand for actual objects/text. A representative **accepted** response shape is:

```json
{
  "answers": [
    { "type": "choice", "name": "motion", "choice": "forward", "confidence": 0.62 },
    { "type": "predicate", "name": "hazard", "probability": 0.10 },
    { "type": "choice", "name": "reason", "choice": "progress", "confidence": 0.81 }
  ],
  "usage": { "input_tokens": 1234 }
}
```

This is a source-derived example, not a record of an actual model call. `choice` is a string; hazard is a numeric `probability`, not a boolean `answer`. The motion and reason confidences must be finite numbers within `[0,1]`; so must hazard probability. The adapter accepts exactly two or three answers: motion and hazard are required, reason is optional for backward compatibility. A missing reason becomes `not_provided`. Extra/unknown/duplicate answers, refusal, unknown action, wrong types or invalid probabilities produce a stop. It does not impose an exact allowed-key set on each answer object or the top-level provider response.

The selected motion's confidence is **informational**: it neither vetoes low-confidence movement nor scales speed. Movement is vetoed if hazard probability is **greater than or equal to 0.25**. A selected `stop` is accepted regardless of the hazard score after basic type/range validation. These model scores are not calibrated real-world safety probabilities.

## 5. Actuation and safeguards

`decisions_adapter.gd:3–9,72–94`, `main.gd:468–560,652–707,1003–1034`:

| Provider motion | Local enum | target_v m/s | target_w rad/s |
|---|---|---:|---:|
| stop | stop | 0 | 0 |
| forward | forward | +0.22 | 0 |
| reverse | reverse | −0.22 | 0 |
| reverse_left | reverse_left | −0.22 | +0.65 |
| reverse_right | reverse_right | −0.22 | −0.65 |
| turn_left | rotate_left | 0 | +0.65 |
| turn_right | rotate_right | 0 | −0.65 |

These are target velocities, not guaranteed measured physical velocities. Reverse left/right describe **nose yaw**, not which way the rear path steers. Nominal 500 ms limits correspond to 0.11 m straight translation and 0.325 rad (~18.6°) turn at the target rate; actual motion depends on drive dynamics, shortened expiry and guards.

1. Native HTTP requests have a 2 s timeout, but the adapter accepts motion only when observation age is strictly **less than 1,000 ms**. A reply at 1.5 s can be a successful API response and still produce a local stale-response stop.
2. Response must match pending `session_id`, `mission_version`, `observation_seq`, `captured_at_ms`; current scan sequence may be newer than the captured one. `mode` must be `live` or `dry_run`. Unrelated responses cannot consume the pending request.
3. Expiry is `min(now+500, captured_at_ms+1000)`. Duration is the remaining interval, bounded below by 0.001 s. Thus an accepted 800 ms-latency response can move for only ~200 ms, not 500 ms.
4. The adapter tags the command with the **current** scan sequence. The local supervisor then requires that sequence to equal the current one, lidar age ≤0.45 s, valid enum, finite duration `(0,1]` s and finite expiry from now through at most now+2 s. Mission version must match when present.
5. A newly accepted semantic plan increments `plan_revision`. Decisions captures the current revision at dispatch; if it changed before the response, that response becomes a stop. Found/blocked status and missing grounded navigation prevent translation independently of model intent.
6. Every non-stop command must pass a sampled swept-footprint check. Straight movement checks its advancing hemisphere; turning/arcing checks all 180 rays. Required rays must be finite and within `[0.15,6]`, so `-1` fails safety even though it is a valid observation encoding. The footprint uses half-width 0.145 m, half-length 0.155 m, plus 0.05 m padding and ≤50 ms trajectory samples. Lidar offset is included.
7. Swept-path safety is rechecked throughout actuation against current scans. Independent simulation collision-box and rigid-body tilt supervisors can stop movement. The tilt threshold is body-up dot world-up <0.85. Both simulation-time duration and monotonic expiry are watched.
8. Stop, manual override, mission change and cancellation invalidate transport generations/session IDs. Changing the mission cancels old motion and requests, increments mission version, then resumes from fresh observations if AI was previously enabled and local enablement checks permit.

The local action JSON schema at `contracts/action.schema.json` is not the complete effective runtime command contract: the adapter additionally attaches `mission_version`, `model_reason`, `validated_observation`, and expiry metadata as applicable, and `submit_action` performs its own checks rather than applying that file's `additionalProperties:false` schema.

## 6. Historical transport validation and memory

### Planner transport validation

`native_decisions_client.gd:530–631`: generations must match; HTTP result must succeed with code 200. Provider response status must be `completed`, `error` and `incomplete_details` null, and `output` an array. Reasoning output items are skipped. Other output items must be completed assistant messages. Exactly one `output_text` content part is allowed; refusal or any other content type fails. The text must parse as a dictionary and satisfy the strict local schema validator, including finite values, types, required fields, bounds and no extra fields. The semantic layer then checks matching capture/session/mission, freshness, allowed dynamic goal, current reachability, evidence uniqueness and found criteria.

### Session memory

Room hypotheses are model-derived, explicitly tagged `visual_hypothesis_not_ground_truth`. Each stores `room_type`, `confidence`, `evidence`, `observed_seq`, `x`, `z`, `kind`. Up to 12 survive in RAM. Visits are appended after >0.45 m from the last saved visit, rounded to 0.01 m, with at most 60 entries. Up to six recent Decisions actions are kept. No conversation/previous-response ID is passed to the API; relevant memory is resent as explicit state.

A mission change intentionally preserves room hypotheses and visits while resetting the plan. App restart does not reload semantic memory from disk. **Historical v0.5 Reset caveat (not a claim about v07.3):** `main.gd:627–650` resets the robot, mapper and recent actions, but does not call `semantic_planner.reset()`. Existing in-process semantic hypotheses/visits/subgoal can therefore survive the UI reset and refer to the pre-reset map. For this historical version, the audit did not establish that “Reset” cleared all planning memory. `SemanticPlanner.reset()` itself also does not reset `plan_revision`; revisions continue across mission resets.

### Excluded account-specific controls

The original audit also described local account enablement, spending enforcement and persistence. That material is omitted from this public historical edition. These omissions are editorial; they do not claim the historical application lacked such controls. No account records, credentials, owner-specific allowance or cost formulas are reproduced here.

## 7. What this implementation does not establish

- It is not full SLAM: encoder translation, noisy IMU yaw and bounded local lidar scan matching lack loop closure, global relocalization and an expanding map.
- It is not a semantic house reconstruction: sparse reachable viewpoint heuristics and short visual hypotheses do not create verified room boundaries, identities or a topological room graph.
- There is no independently calibrated vision confidence or real-world safety probability. `found` uses the planner's declared evidence.
- Simulator collision/tilt supervision and swept-footprint geometry are heuristics, not a hardware safety certification or calibrated stopping-distance proof.
- Static reading establishes implementation contracts; it does not prove live provider availability, current API compatibility, installed-binary equivalence or task success.

## 8. Exact instruction text from the implementation

The following appendix is extracted directly from the source constants/string literals. There is no separate system/developer prompt.

### MOTION_INSTRUCTIONS (`decisions_adapter.gd`)

```text
Make measurable progress toward the current task using one bounded 500 ms action. Choose forward or a turn when supported by observed clearance; reverse can reposition from a dead end only with fresh rear lidar clearance. Stop when the task is complete, all useful motions are blocked, or evidence is insufficient. Do not stop merely because the map is incomplete: unknown cells are not free, but observed free space can support a bounded step. turn_left/right rotate in place. reverse_left/right move backward while the ROBOT NOSE yaws left/right (positive/negative yaw); these are not rear-path steering labels. Use controller_context reverse flags, measured directional clearances, recent action outcomes and map_progress to avoid repeating a stopped or ineffective choice. Observed grid fraction is not mission completion. If a useful observed-safe bounded motion exists, prefer progress or repositioning; never fabricate clearances or completion.
```

### REASON_INSTRUCTIONS (`decisions_adapter.gd`)

```text
Classify the reason for the selected motion: progress toward task; reposition for a better route/view; obstacle blocks useful motion; insufficient_evidence for any safe useful motion; mission_complete only when observed evidence demonstrates completion.
```

### Planner prefix (`native_decisions_client.gd:447`)

```text
You are a bounded semantic navigation planner, never a motor controller. Use only the supplied camera, measured occupancy map, and sensor state. Never invent hidden objects, coordinates, free space, completion, or sensor evidence. Unknown map cells are not free. Follow the mission and planning contract in state.planner_context. Return only the requested structured plan; when evidence is insufficient request a safe observation or report blocked. 
```

### Base observation text (`native_decisions_client.gd:292`)

```text
Bounded mobile-robot navigation. First image: robot camera. Second: measured local occupancy map. Unknown cells are not free. State: [JSON-serialized projected sensor state]
```

### Planner policy (`semantic_planner.gd:161`)

```text
Choose only a supplied goal_id or look_here. Candidate coordinates are reachable observed-free positions, not room labels. Classify current POV only; previous hypotheses are uncertain. Search unvisited frontier viewpoints before repeating visited positions. Use inspect/look_here only when rotating could resolve current visual evidence or no reachable waypoint exists. Find bedroom succeeds only when bed is visible now. Never infer completion from map coverage. No house truth is provided.
```

### Controller instruction (`semantic_planner.gd:252–266`)

```text
Follow the observed-free waypoint using bounded motion and local lidar guards. A room hypothesis is not a known room. Never claim mission complete without planner visual confirmation.
```

### Controller instruction (`semantic_planner.gd:252–266`)

```text
Stop if found or blocked. Otherwise rotate safely to inspect the current view and discover observed free space; do not translate without a grounded waypoint.
```

### Controller instruction (`semantic_planner.gd:252–266`)

```text
Stop; observed route changed and requires replanning.
```

## 9. Editorial provenance

This edition retains the historical architecture, scheduling, observation projection, units, schema, candidate/route heuristics, response validation, safeguards, memory behavior, known reset caveat and exact application instruction strings. Provider-specific transport identifiers and owner/account financial context were removed. No behavior was retested and no implementation was changed in creating this document. Use the public v07.3 subset's own README and verification document for its current inclusion and test boundaries.
