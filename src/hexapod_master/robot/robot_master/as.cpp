    {
        Jacobian<<
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM1*sin(angle_(0)) + LEG_DH_PARAM2*cos(angle_(0))*cos(angle_1)),
                - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*sin(angle_(0))*sin(angle_(1)),
                -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)),

                - LEG_DH_PARAM1*cos(angle_(0)) - LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*cos(angle_(1))*sin(angle_(0)),
                - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM2*cos(angle_(0))*sin(angle_(1)),
                -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0))   ,

                0,                           
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) + LEG_DH_PARAM2*cos(angle_(1)),              
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2));
    }
    else
    {
        Jacobian<<
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM1*sin(angle_(0)) + LEG_DH_PARAM2*cos(angle_(0))*cos(angle_(1)) ),
                -( - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*sin(angle_(0))*sin(angle_(1)) ),
                -( -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) ),

                ( - LEG_DH_PARAM1*cos(angle_(0)) - LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*cos(angle_(1))*sin(angle_(0)) ),
                ( - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM2*cos(angle_(0))*sin(angle_(1)) ),
                ( -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0))   ),

                -0,                           
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) + LEG_DH_PARAM2*cos(angle_(1)) ),              
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) );
    }