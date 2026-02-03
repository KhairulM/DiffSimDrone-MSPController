#!/bin/bash
# Optimization script for MangoPi to improve inference performance

echo "Setting CPU governor to performance mode..."
# Check if we have permissions
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo for best performance"
    echo "Usage: sudo ./optimize_mangopi.sh"
    exit 1
fi

# Set CPU governor to performance mode (prevents frequency scaling)
for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
    if [ -f "$cpu/cpufreq/scaling_governor" ]; then
        echo performance > "$cpu/cpufreq/scaling_governor"
        echo "Set $(basename $cpu) to performance mode"
    fi
done

# Disable CPU idle states for consistent performance (optional, increases power consumption)
# Uncomment if you need maximum consistency
# echo 1 > /sys/devices/system/cpu/cpu0/cpuidle/state1/disable
# echo 1 > /sys/devices/system/cpu/cpu0/cpuidle/state2/disable

echo ""
echo "Optimization applied! Performance should be more consistent now."
echo "Note: This increases power consumption and heat generation."
echo ""
echo "To revert to power-saving mode later, run:"
echo "  for cpu in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do echo ondemand > \$cpu; done"
