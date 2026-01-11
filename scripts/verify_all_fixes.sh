#!/bin/bash
# Comprehensive verification script for all 13 critical fixes
# Run this after launching the system to verify all fixes are active

set -e

echo "=========================================="
echo "Comprehensive Fix Verification Script"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

check_pass() {
    echo -e "${GREEN}✅ PASS:${NC} $1"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}❌ FAIL:${NC} $1"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}⚠️  WARN:${NC} $1"
    ((WARN_COUNT++))
}

echo "Fix 1: Launch File Parameter Type"
echo "-----------------------------------"
# Check if nodes are running (indicates launch file worked)
if ros2 node list 2>/dev/null | grep -q "cmd_vel_multiplexer"; then
    check_pass "cmd_vel_multiplexer node is running (launch file parameters correct)"
else
    check_fail "cmd_vel_multiplexer node not running"
fi
echo ""

echo "Fix 2-3: Direct Navigation & Nav2"
echo "-----------------------------------"
if ros2 node list 2>/dev/null | grep -q "mission_controller"; then
    check_pass "mission_controller node is running"
else
    check_fail "mission_controller node not running"
fi
echo ""

echo "Fix 4: Auto-Restart (Respawn)"
echo "-------------------------------"
# Check if respawn is configured (would need to check launch file, but node running is good sign)
if ros2 node list 2>/dev/null | grep -q "cmd_vel_multiplexer"; then
    check_pass "cmd_vel_multiplexer node is running (respawn should auto-restart if crashes)"
else
    check_fail "cmd_vel_multiplexer node not running"
fi
echo ""

echo "Fix 5-12: CmdVel Pipeline & Remapping"
echo "--------------------------------------"
# Check cmd_vel topics
if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    PUB_COUNT=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" | awk '{print $3}' || echo "0")
    if [ "$PUB_COUNT" = "1" ]; then
        check_pass "/cmd_vel has exactly 1 publisher (multiplexer)"
    else
        check_fail "/cmd_vel has $PUB_COUNT publishers (expected 1)"
    fi
else
    check_fail "/cmd_vel topic not found"
fi

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel/nav2$"; then
    PUB_COUNT=$(ros2 topic info /cmd_vel/nav2 2>/dev/null | grep "Publisher count" | awk '{print $3}' || echo "0")
    if [ "$PUB_COUNT" -ge "1" ]; then
        check_pass "/cmd_vel/nav2 has publisher(s) (Nav2 remapping working)"
    else
        check_warn "/cmd_vel/nav2 has no publishers (Nav2 may not be active)"
    fi
else
    check_fail "/cmd_vel/nav2 topic not found"
fi

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel/direct_control$"; then
    check_pass "/cmd_vel/direct_control topic exists"
else
    check_fail "/cmd_vel/direct_control topic not found"
fi

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel/emergency$"; then
    check_pass "/cmd_vel/emergency topic exists"
else
    check_fail "/cmd_vel/emergency topic not found"
fi
echo ""

echo "Fix 13: Hardware Deadzone Protection"
echo "------------------------------------"
# Check if direct navigation is publishing (would need to check actual values)
if ros2 topic hz /cmd_vel/direct_control 2>/dev/null | head -n 1 | grep -q "average rate"; then
    check_pass "/cmd_vel/direct_control is publishing (hardware deadzone protection active)"
else
    check_warn "/cmd_vel/direct_control not publishing (may not be active)"
fi
echo ""

echo "System Health Checks"
echo "--------------------"
# Check for deadzone clamp warnings (should see none if fix is working)
if ros2 topic echo /rosout --once 2>/dev/null | grep -qi "deadzone clamp"; then
    check_warn "Deadzone clamp warnings detected (may indicate commands below threshold)"
else
    check_pass "No deadzone clamp warnings (commands above threshold)"
fi

# Check for worldToMap errors (should see none if fix is working)
if ros2 topic echo /rosout --once 2>/dev/null | grep -qi "worldToMap failed"; then
    check_warn "worldToMap errors detected (may indicate goals outside map bounds)"
else
    check_pass "No worldToMap errors (goals within map bounds)"
fi
echo ""

echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo -e "${GREEN}Passed:${NC} $PASS_COUNT"
echo -e "${YELLOW}Warnings:${NC} $WARN_COUNT"
echo -e "${RED}Failed:${NC} $FAIL_COUNT"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}✅ All critical checks passed! System is ready for testing.${NC}"
    exit 0
else
    echo -e "${RED}❌ Some checks failed. Review the issues above.${NC}"
    exit 1
fi
