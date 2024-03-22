

action_dic =    {
                    'start_curl':  [ 
                        {"motor1": 1423, "motor2": 2672, "motor3": 1025, "motor4": 2672, "motor5": 1423},
                        {"motor1": 1423, "motor2": 2672, "motor3": 1923, "motor4": 2672, "motor5": 1423}
                    ],
                                
                    'get_down_right': [
                    #action 1    
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand
                        # {'motor1': 1535, 'motor2': 2564, 'motor3': 1022, 'motor4': 1649, 'motor5': 2424},#sit
                        # {'motor1': 1535, 'motor2': 2564, 'motor3': 1022, 'motor4': 3875, 'motor5': 2424},#shaek left back-end feet
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 3875, 'motor5': 1525},#right back-end feet back
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand
                        # {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659}#curl
                    
                    #action 2
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand

                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},
                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},
                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},
                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},
                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},
                        {'motor1': 1427, 'motor2': 2097, 'motor3': 1452, 'motor4': 1820, 'motor5': 1380},

                        {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659}#curl


                    ],

                    'get_down_left': [ 
                    #action 1    
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#stand
                        # {'motor1': 1535, 'motor2': 2564, 'motor3': 1047, 'motor4': 1649, 'motor5': 2424},#sit
                        # {'motor1': 1535, 'motor2': 2564, 'motor3': 1047, 'motor4': 1649, 'motor5': 198 },#shaek right back-end feet
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 198 },#left back-end feet back
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#stand
                        # {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659}#curl

                    #action 2
                        #action 2
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand

                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},
                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},
                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},
                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},
                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},
                        {'motor1': 2005, 'motor2': 2676, 'motor3': 1452, 'motor4': 2712, 'motor5': 2272},

                        {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659}#curl

                    ],

                    'stand_up_from_right': [
                        # {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659},#curl
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand
                        # {'motor1': 1535, 'motor2': 3612, 'motor3': 1022, 'motor4': 2567, 'motor5': 2263},#back
                        # {'motor1': 1535, 'motor2': 2567, 'motor3': 1022, 'motor4': 2567, 'motor5': 1525},#stand

                        {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659},#curl
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#stand
                        {'motor1': 1690, 'motor2': 3659, 'motor3': 1021, 'motor4': 2514, 'motor5': 2268},
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#back

                    ],

                    'stand_up_from_left': [
                        {'motor1': 1624, 'motor2': 2404, 'motor3': 1929, 'motor4': 2508, 'motor5': 1659},#curl
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#stand
                        {'motor1': 490 , 'motor2': 2567, 'motor3': 1011, 'motor4': 1829, 'motor5': 1525},#back
                        {'motor1': 1535, 'motor2': 2567, 'motor3': 1021, 'motor4': 2567, 'motor5': 1525},#stand
                    ]
                }