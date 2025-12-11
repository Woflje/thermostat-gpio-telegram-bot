#/bin/sh

screenid="thermostat"
screen -d -m -S $screenid python main.py
screen -S $screenid -X multiuser on
screen -r $screenid
