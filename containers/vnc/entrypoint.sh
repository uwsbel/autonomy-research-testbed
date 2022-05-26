#!/bin/bash
set -ex

exec supervisord -c /opt/supervisord.conf
