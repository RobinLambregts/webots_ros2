#!/usr/bin/env bash

echo "ros killen..."
sudo pkill -f ros
echo "webots killen..."
sudo pkill -f webots

echo "No evidence left behind ;)"