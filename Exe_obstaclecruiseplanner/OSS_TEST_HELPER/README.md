# OSS_TEST_HELPER

Minimal CLI tool that reads ROS-style `Trajectory` jsonl records from the bundled `reference_output/scenario_*` folders, rewrites their timestamps at random 150~200 ms intervals, and persists the emitted messages into new jsonl files.

## Building

1. `mkdir -p build && cd build`
2. `cmake ..`
3. `cmake --build .`

The project requires a C++17 compiler and embeds the single-header `nlohmann/json.hpp` under `include/vendor` so no extra install is needed.

## Running

From the build directory run `./OSS_TEST_HELPER`. The process waits for user commands:

- `scenario_1`, `scenario_2`, `scenario_3`: trigger the corresponding jsonl processor
- `exit` or `quit`: stop the loop

Each scenario:

1. Identifies the `planning__scenario_planning...jsonl` file in `reference_output/scenario_X`
2. Loads every line into an internal `Trajectory` struct
3. Emits each record through a random 150~200 ms gap while updating the `timestamp` and `message.header.stamp`
4. Logs `index`, `timestamp`, and `points` count
5. Saves the rewritten stream to `output/<original_stem>_scenario_X.jsonl`

## Validation checklist

1. Scenario commands read the jsonl source line-by-line and parse each entry.
2. The output jsonl contains the same number of lines that were present in the input.
3. Timestamps increase monotonically by 150–200 ms between consecutive messages.
4. `message.header.stamp.sec`/`nanosec` mirror the emitted `timestamp`.
5. The output filename appends `_scenario_X` to the original base name and lands in `output/`.
## Output format guarantees

- Each emitted line reorders keys to match the reference layout (top-level `topic`, `timestamp`, `message`; header stamp before `frame_id`; consistent field ordering inside each trajectory point) while keeping the message shrunk to a single line.
- The serializer emits `": "` and `", "` separators, stabilizes numeric precision, and double-checks that the first three lines start with `{"topic":`, contain the expected whitespace, and expose the top-level key order.
- After a scenario run, the processor echoes the first 200 characters of the reference and generated first lines so you can visually confirm the serialized string, while the new validator also enforces `"header": {"stamp": {"sec": …, "nanosec": …}, "frame_id": …}` ordering.
