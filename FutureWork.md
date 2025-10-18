## Next Release
- Release 0.1.2
    - General
        - [ ] Fix map to pose conversions
    - GUI
        - [ ] Change behavior of the pen and eraser to support dragging movements
    - Bug Fixes/Optimizations
        - [ ] Add more extensive error handling for CLI parameters
        - [ ] Investigate PRM path generation sometimes resulting in collisions (usually occurs with small sample count)
        - [ ] Optimize algorithms   
            - [ ] convert map to unordered_map
            - [ ] PRM find_nearest_neighbors()
            - [ ] Implement Priority queue for A*
    - Testing
        - [ ] Confirm map conversion tests work properly

# Backlog
 - General
 - Algorithms
    - Implement D* Lite (CLI & GUI)
 - CLI
 - GUI
    - [Optional] Animate traversal of map and final path
 - Testing
    - Test D* Replan with changing map
 - Bug fixes/Optimizations
    - Add more extensive error handling for GUI