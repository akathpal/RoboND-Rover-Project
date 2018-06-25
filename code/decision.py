import numpy as np
import cv2

def rock_picking(Rover):
    if Rover.near_sample and not Rover.picking_up:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
                # pick up the rock
        if Rover.vel == 0:
            Rover.send_pickup = True
                #Rover.throttle = -0.2
                #Rover.brake = 0
    elif Rover.picking_up and not Rover.near_sample:
        Rover.brake = 0
        Rover.send_pickup = False
        Rover.mode = 'forward'
    elif Rover.picking_up and Rover.near_sample:
        cv2.putText(Rover.vision_image,'Collecting sample',(0, 70),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
        Rover.throttle = -0.2
        Rover.steer = 0
        Rover.brake = 0
        return Rover
    #rd = Rover.rock_dist > 150
    #Rover.rock_angle[rd] = np.mean(Rover.rock_angle)
    if (len(Rover.rock_angle) >= 5):
        
        y = np.mean(Rover.rock_angle * 180/np.pi)
        Rover.steer = np.clip(y, -15, 15)
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
        if np.mean(Rover.rock_dist) < 40:
            Rover.brake = Rover.brake_set + 5
            Rover.throttle = 0
            if Rover.near_sample is not True:
                cv2.putText(Rover.vision_image,'Searching for sample',(0, 65),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
                Rover.throttle = 0
                Rover.brake = 0
                while(Rover.near_sample is False):
                    Rover.steer = -15
                
        #print('Rock_angle =' + str(Rover.rock_angle),'Rock_dist = '+ str(Rover.rock_dist))    
    else:
        return Rover
    return Rover

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    #a = np.mean(Rover.nav_angles * 180/np.pi)
    #y = np.min(Rover.nav_angles * 180/np.pi)
    #z = np.max(Rover.nav_angles * 180/np.pi)
    # Example:
    # Check if we have vision data to make decisions with
    #cv2.putText(Rover.vision_image,' mean = '+str(int(a))+' min = '+ str(int(y)) + ' max = '+str(int(z)),(0, 45),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
    if (Rover.nav_angles is not None): #& (np.mean(Rover.nav_dists) > 20):
    #if len(Rover.nav_angles) > 20:
        
        dists = Rover.nav_dists < 120
        #Rover.nav_angles[dists] = np.mean(Rover.nav_angles)
        x = np.mean(Rover.nav_angles[dists] * 180/np.pi)  
        #cv2.putText(Rover.vision_image,'go_angle = ' + str(int(x)),(0, 85),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
        #if (Rover.vel <= 0.2) & (np.absolute(Rover.yaw - x)>20):
         #   cv2.putText(Rover.vision_image,"STUCK",(65, 25),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
         #   Rover.steer = x - (Rover.yaw)
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(x, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(x, -15, 15)
                    Rover.mode = 'forward'
        Rover = rock_picking(Rover)
        
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        #cv2.putText(Rover.vision_image,'Nowhere to go',(0, 65),cv2.FONT_HERSHEY_COMPLEX,0.45, (255,255,255), 1)
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0


    return Rover



