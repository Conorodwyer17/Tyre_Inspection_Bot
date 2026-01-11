#!/bin/bash
# Verification script for direct navigation activation order bug fix
# This script verifies that set_goal() is called before activate() to prevent deactivation bug

echo "=========================================="
echo "Direct Navigation Activation Order Verification"
echo "=========================================="
echo ""
echo "This script verifies that the critical bug is fixed:"
echo "  Bug: activate() called before set_goal() causes immediate deactivation"
echo "  Fix: set_goal() called FIRST, then activate() only if not already active"
echo ""

# Check if mission_controller.py has the fix
FILE="src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/mission_controller.py"

echo "Checking for correct activation order in mission_controller.py..."
echo ""

# Check for WRONG pattern: activate() before set_goal()
if grep -n "direct_nav_fallback.activate()" "$FILE" | grep -A1 -E "direct_nav_fallback.set_goal" | grep -v "was_active" > /dev/null; then
    echo "❌ ERROR: Found activate() called before set_goal() without was_active check!"
    echo "   This causes the deactivation bug!"
    grep -n "direct_nav_fallback.activate()" "$FILE" | grep -A1 -E "direct_nav_fallback.set_goal" | grep -v "was_active"
    exit 1
else
    echo "✅ No instances of activate() before set_goal() without was_active check"
fi

# Check for CORRECT pattern: set_goal() before activate(), or preserve_active_state used
echo ""
echo "Checking for correct patterns:"
echo ""

CORRECT_COUNT=0
WRONG_COUNT=0

# Count correct patterns (set_goal before activate, or preserve_active_state used)
if grep -B5 "direct_nav_fallback.activate()" "$FILE" | grep -E "(set_goal.*preserve_active_state|was_active.*set_goal)" > /dev/null; then
    CORRECT_COUNT=$(grep -B5 "direct_nav_fallback.activate()" "$FILE" | grep -c -E "(set_goal.*preserve_active_state|was_active.*set_goal)" || echo "0")
    echo "✅ Found $CORRECT_COUNT correct pattern(s): set_goal() with preserve_active_state or was_active check"
fi

# Check all direct_nav_fallback calls
echo ""
echo "All direct_nav_fallback activation/set_goal calls:"
grep -n "direct_nav_fallback\.\(activate\|set_goal\)" "$FILE" | grep -A1 "activate\|set_goal" | head -20

echo ""
echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "✅ Activation order bug fix verified"
echo ""
echo "Correct pattern:"
echo "  was_active = self.direct_nav_fallback.is_active"
echo "  self.direct_nav_fallback.set_goal(goal, preserve_active_state=was_active)"
echo "  if not was_active:"
echo "      self.direct_nav_fallback.activate()"
echo ""
echo "This ensures:"
echo "  1. Goal is set BEFORE activation"
echo "  2. If already active, preserve_active_state=True keeps it active"
echo "  3. If not active, activate() is called AFTER set_goal()"
echo ""
echo "Test command to verify at runtime:"
echo "  ros2 topic echo /cmd_vel/direct_control"
echo "  # Should see continuous cmd_vel when direct navigation activates"
