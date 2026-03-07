# Driver Controller Bindings

| Button | Command | Description | Activation |
| :--- | :--- | :--- | :--- |
| **A** | Lock Heading (Bump) | Locks robot heading to 45° for bump crossing, limits to bump speed | Toggle |
| **B** | Calibrate Gyro | Zeros the gyro, preserves field position | Instant |
| **X** | Hub Arc Lock | Locks robot at launching range and orbits hub (Launch zone only) | Toggle |
| **Y** | Toggle Field Relative | Switch between field-relative and robot-relative drive | Instant |
| **Right Bumper** | Index Forward | Feeds fuel toward launcher | Toggle |
| **Left Bumper** | Index Reverse | Reverses indexer and treadmill | Toggle |
| **Right Trigger** | Drive & Auto Launch | Tracks hub, auto-launches when on target (Launch zone only) | Hold |
| **Left Trigger** | Lock Launching Range | Toggles radial distance lock while `driveAndAutoShootCmd` is active | Toggle |
| **D-Pad Up** | Launcher 2750 RPM | Spins launcher at 2750 RPM | Toggle |
| **D-Pad Down** | Launcher 1000 RPM | Spins launcher at 1000 RPM | Toggle |
| **D-Pad Left** | Pull Left Climber | Manual left climber, speed from (Left Trigger − Right Trigger) × 0.3 | Hold |
| **D-Pad Right** | Pull Right Climber | Manual right climber, speed from (Left Trigger − Right Trigger) × 0.3 | Hold |
| **Left Stick (Click)** | Slow Mode | Reduces translation speed to 30% while held | Hold |
| **Right Stick (Click)** | Full Turn Speed | Overrides turn sensitivity from 40% to 100% while held | Hold |
| **Left Stick (Axes)** | Drive | Field-relative translation | Analog |
| **Right Stick (X-Axis)** | Rotate | Robot rotation (40% sensitivity default) | Analog |
| **Back (Button 7)** | Cancel Drive Command | Interrupts active drive command, returns to manual control | Instant |