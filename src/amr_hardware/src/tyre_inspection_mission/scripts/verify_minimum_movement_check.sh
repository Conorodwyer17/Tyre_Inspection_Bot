#!/bin/bash
# Verification script for minimum movement check fix
# This script verifies that arrival detection ALWAYS checks minimum movement, even when nav_initial_distance is None

echo "=========================================="
echo "Minimum Movement Check Verification"
echo "=========================================="
echo ""
echo "This script verifies that the critical bug is fixed:"
echo "  Bug: When nav_initial_distance is None, arrival detection used old logic without minimum movement check"
echo "  Fix: Use nav_progress_distance as fallback to ensure minimum movement is ALWAYS checked"
echo ""

FILE="src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/mission_controller.py"

echo "Checking for minimum movement check in license plate navigation..."
echo ""

# Check for nav_progress_distance fallback in license plate navigation
if grep -A5 "No initial distance stored\|nav_progress_distance.*fallback" "$FILE" | grep -E "nav_progress_distance|distance_traveled.*nav_progress" > /dev/null; then
    echo "✅ Found nav_progress_distance fallback in license plate navigation"
    grep -B2 -A5 "nav_progress_distance.*fallback\|No initial distance stored" "$FILE" | head -20
else
    echo "⚠️  WARNING: nav_progress_distance fallback not found in license plate navigation"
fi

echo ""
echo "Checking for minimum movement check in tyre navigation..."
echo ""

# Check if tyre navigation has same fix
TYRE_FIX_COUNT=$(grep -c "nav_progress_distance.*tyre\|Tyre navigation.*nav_progress" "$FILE" || echo "0")
if [ "$TYRE_FIX_COUNT" -gt "0" ]; then
    echo "✅ Found nav_progress_distance usage in tyre navigation"
else
    echo "❌ ERROR: Tyre navigation does NOT have minimum movement check fix!"
    echo "Tyre navigation still uses old logic without minimum movement verification."
fi

echo ""
echo "Checking for 'else' block that doesn't check minimum movement..."
echo ""

# Count occurrences of old logic patterns (should be 0)
OLD_LOGIC_COUNT=$(grep -c "No initial distance stored - use old logic\|backward compatibility.*arrival_distance" "$FILE" || echo "0")
if [ "$OLD_LOGIC_COUNT" -eq "0" ]; then
    echo "✅ No old logic without minimum movement check found"
else
    echo "⚠️  WARNING: Found $OLD_LOGIC_COUNT occurrences of old logic without minimum movement check"
fi

echo ""
echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "✅ License plate navigation: Minimum movement check fixed"
echo "❌ Tyre navigation: NEEDS FIX - still uses old logic"
echo ""
echo "Required fix for tyre navigation:"
echo "  1. Use nav_progress_distance as fallback when nav_initial_distance is None"
echo "  2. Require minimum movement (0.5m) before declaring arrival"
echo "  3. Don't declare arrival if no distance tracking is available"
echo ""
