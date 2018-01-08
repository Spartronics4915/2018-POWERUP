#!/bin/bash
echo "Checking code formatting."
java -jar google-java-format-1.5-all-deps.jar --set-exit-if-changed --dry-run ../../src/com/spartronics4915/frc2018/* src/com/spartronics4915/frc2018/*/* src/com/spartronics4915/frc2018/*/*/* >/dev/null 2>&1
CODE=$?
if [ "$CODE" -eq "0" ]; then
	echo "Formatting is OK."
else
	echo "Formatting check failed! Run formatter.bat (Windows) or formatter.sh."
fi
exit $CODE
