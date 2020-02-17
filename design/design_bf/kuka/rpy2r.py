import numpy as np
import math


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


_T = np.array([0.955924 ,-0.24321 ,0.164495 ,0 ,0.164495 ,0.907673 ,0.386096 ,2.819078 ,-0.24321 ,-0.34202 ,0.907673 ,24.97394 ,0 ,0 ,0 ,1]).reshape(4, 4)
_R = _T[0:3, 0:3]

print(rotationMatrixToEulerAngles(_R))
