python -m lerobot.teleoperate \
    --robot.ip_address_left="localhost" \
    --robot.port_left=12346 \
    --robot.ip_address_right="localhost" \
    --robot.port_right=12345 \
    --robot.type=enpei_follower \
    --robot.id=enpei_follower \
    --robot.cameras="{ }" \
    --teleop.type=enpei_leader \
    --teleop.port_left=/dev/ttyACM1 \
    --teleop.port_right=/dev/ttyACM0 \
    --teleop.id=enpei_leader \
    --fps=30\
    --display_data=false \
    --enpei_speed_mode=record
    
    