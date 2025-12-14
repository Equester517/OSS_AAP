# Rollback Instructions

## Current Version
- Simple analytical optimization (no OSQP)
- Target: 30cm position error

## To Rollback to OSQP Version:
```bash
cd /home/bskang/autoware/src/universe/autoware.universe/planning/Path_Optimizer

# Restore OSQP implementation
cp src/mpt_optimizer.cpp.osqp_backup src/mpt_optimizer.cpp
cp src/osqp_interface.cpp.backup src/osqp_interface.cpp

# Rebuild
cd build && make -j$(nproc) && cd ..

# Test
./run_both_scenarios.sh
python3 compare_ros2_vs_standalone.py
```

## Backup Files:
- `src/mpt_optimizer.cpp.osqp_backup` - OSQP version
- `src/osqp_interface.cpp.backup` - OSQP interface
- `src/mpt_optimizer.cpp.simple_backup` - Simple version (if created)
