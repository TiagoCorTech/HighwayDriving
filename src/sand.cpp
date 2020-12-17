/*
                car_speed = (car_speed + car_a*dt) > maxv ? maxv : car_speed + car_a*dt  ;
                car_speed = car_speed < 0 ? 0 : car_speed;
				

                vector<double> car_frenet = getFrenet(car_x,car_y,rad2deg(car_yaw),map_waypoints_x,map_waypoints_y);

                next_s = car_frenet[0] + car_speed*dt;
                next_d = line2d(d2line(car_frenet[1]));
                
                vector<double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);

                car_x = xy[0];
                car_y = xy[1];
                */