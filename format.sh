#!/bin/bash
for i in 1 2 # This works around a bug where the formatter sometimes misses a file
do
  java -jar test/format/google-java-format-1.5-all-deps.jar -r src/com/spartronics4915/*/* src/com/spartronics4915/*/*/* src/com/spartronics4915/*/*/*/*
done
