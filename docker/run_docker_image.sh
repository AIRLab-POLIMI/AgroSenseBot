#!/bin/bash

USER_NAME=asb

sudo docker run -ti --rm --network=host -v ~/asb_logs:/home/"$USER_NAME"/asb_logs -v ~/w:/home/"$USER_NAME"/w asb:v3
