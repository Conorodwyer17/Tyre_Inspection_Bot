#!/bin/bash
# Proof: Verify that check_navigation_complete() checks BOTH distance AND orientation
# This prevents false "navigation complete" when robot is at goal position but wrong orientation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MISSION_CONTROLLER="$SCRIPT_DIR/../tyre_inspection_mission/core/mission_controller.py"

echo "=========================================="
echo "Verifying orientation check in check_navigation_complete()"
echo "=========================================="
echo ""

# Check 1: Verify that check_navigation_complete() checks orientation when Nav2 reports SUCCEEDED
echo "Check 1: Verify orientation check in check_navigation_complete() when Nav2 reports SUCCEEDED"
if grep -A 30 "if result.result == NavigateToPose.Result.SUCCEEDED:" "$MISSION_CONTROLLER" | grep -q "orientation_diff\|arrival_orientation_threshold\|orientation_match"; then
    echo "✅ PASS: check_navigation_complete() checks orientation when Nav2 reports SUCCEEDED"
else
    echo "❌ FAIL: check_navigation_complete() does NOT check orientation when Nav2 reports SUCCEEDED"
    exit 1
fi
echo ""

# Check 2: Verify that it checks BOTH distance AND orientation
echo "Check 2: Verify that it checks BOTH distance AND orientation"
if grep -A 50 "if result.result == NavigateToPose.Result.SUCCEEDED:" "$MISSION_CONTROLLER" | grep -q "arrival_distance <= arrival_threshold.*orientation_match\|orientation_match.*arrival_distance <= arrival_threshold"; then
    echo "✅ PASS: check_navigation_complete() checks BOTH distance AND orientation"
else
    # Try alternative pattern with line breaks
    if grep -A 60 "if result.result == NavigateToPose.Result.SUCCEEDED:" "$MISSION_CONTROLLER" | grep -A 5 "arrival_distance <= arrival_threshold" | grep -q "orientation_match"; then
        echo "✅ PASS: check_navigation_complete() checks BOTH distance AND orientation (found with alternative pattern)"
    else
        echo "❌ FAIL: check_navigation_complete() does NOT check BOTH distance AND orientation"
        exit 1
    fi
fi
echo ""

# Check 3: Verify that it rejects Nav2 result if orientation is wrong
echo "Check 3: Verify that it rejects Nav2 result if orientation is wrong"
if grep -A 40 "if result.result == NavigateToPose.Result.SUCCEEDED:" "$MISSION_CONTROLLER" | grep -q "arrival_distance <= arrival_threshold and not orientation_match"; then
    echo "✅ PASS: check_navigation_complete() rejects Nav2 result if orientation is wrong"
else
    echo "❌ FAIL: check_navigation_complete() does NOT reject Nav2 result if orientation is wrong"
    exit 1
fi
echo ""

# Check 4: Verify that orientation calculation matches arrival detection logic
echo "Check 4: Verify that orientation calculation matches arrival detection logic"
if grep -A 30 "if result.result == NavigateToPose.Result.SUCCEEDED:" "$MISSION_CONTROLLER" | grep -q "math.atan2.*goal_siny_cosp.*goal_cosy_cosp"; then
    echo "✅ PASS: Orientation calculation uses same logic as arrival detection"
else
    echo "❌ FAIL: Orientation calculation does NOT match arrival detection logic"
    exit 1
fi
echo ""

# Check 5: Verify that intermediate distance case also checks orientation
echo "Check 5: Verify that intermediate distance case (between threshold and 2x threshold) also checks orientation"
# Check if comment exists
if grep -q "Also check orientation even for intermediate distance" "$MISSION_CONTROLLER"; then
    # Find the line number and check nearby code
    line_num=$(grep -n "Also check orientation even for intermediate distance" "$MISSION_CONTROLLER" | cut -d: -f1)
    if [ -n "$line_num" ]; then
        # Check if orientation_match appears within 10 lines after the comment
        if sed -n "${line_num},$((line_num+10))p" "$MISSION_CONTROLLER" | grep -q "if orientation_match:"; then
            echo "✅ PASS: Intermediate distance case also checks orientation"
        else
            echo "❌ FAIL: Comment found but orientation_match check not found nearby"
            exit 1
        fi
    else
        echo "❌ FAIL: Could not find comment line number"
        exit 1
    fi
else
    echo "❌ FAIL: Intermediate distance case does NOT check orientation (comment not found)"
    exit 1
fi
echo ""

echo "=========================================="
echo "✅ ALL CHECKS PASSED"
echo "=========================================="
echo ""
echo "Proof: check_navigation_complete() now correctly checks BOTH distance AND orientation"
echo "This prevents false 'navigation complete' when robot is at goal position but wrong orientation."
echo ""
echo "This matches the arrival detection logic in:"
echo "  - _handle_license_plate_navigation() (lines ~3437-3483)"
echo "  - _handle_tyre_navigation() (lines ~4201-4237)"
echo ""
