q = zeros(1, 6);

DH_PARAMS = [   0.525,  q(1),   0.15,   -pi/2;
                0,      q(2)-pi/2,   0.79,   0;
                0,      q(3),   0.15,   -pi/2;
                0.86,   q(4),   0,      pi/2;
                0,      q(5),   0,    	-pi/2;
                0.1,    q(6),   0,      0
             ];
            
T01 = trvec2tform([0, 0, DH_PARAMS(1,1)])*rotm2tform(rotz(rad2deg(DH_PARAMS(1,2))))*trvec2tform([DH_PARAMS(1,3), 0, 0])*rotm2tform(rotx(rad2deg(DH_PARAMS(1,4))));
T12 = rotm2tform(rotz(rad2deg(DH_PARAMS(2,2))))*trvec2tform([DH_PARAMS(2,3), 0, 0]);
T23 = rotm2tform(rotz(rad2deg(DH_PARAMS(3,2))))*trvec2tform([DH_PARAMS(3,3), 0, 0])*rotm2tform(rotx(rad2deg(DH_PARAMS(3,4))));
T34 = trvec2tform([0, 0, DH_PARAMS(4,1)])*rotm2tform(rotz(rad2deg(DH_PARAMS(4,2))))*rotm2tform(rotx(rad2deg(DH_PARAMS(4,4))));
T45 = rotm2tform(rotz(rad2deg(DH_PARAMS(5,2))))*rotm2tform(rotx(rad2deg(DH_PARAMS(5,4))));
T56 = trvec2tform([0, 0, DH_PARAMS(6,1)])*rotm2tform(rotz(rad2deg(DH_PARAMS(6,2))));

%% BASE_LINK - FLANGE

T06 = T01*T12*T23*T34*T45*T56;

t06 = tform2trvec(T06)
R06 = tform2rotm(T06)
RPY06 = rotm2eul(R06)
aa06 = rotm2axang(R06)

%% LINK1 - FLANGE

T16 = T12*T23*T34*T45*T56;

t16 = tform2trvec(T16)
R16 = tform2rotm(T16)
RPY16 = rotm2eul(R16)
aa16 = rotm2axang(R16)

%% LINK2 - FLANGE

T26 = T23*T34*T45*T56;

t26 = tform2trvec(T26)
R26 = tform2rotm(T26)
RPY26 = rotm2eul(R26)
aa26 = rotm2axang(R26)

%% LINK3 - FLANGE

T36 = T34*T45*T56;

t36 = tform2trvec(T36)
R36 = tform2rotm(T36)
RPY36 = rotm2eul(R36)
aa36 = rotm2axang(R36)

%% LINK4 - FLANGE

T46 = T45*T56;

t46 = tform2trvec(T46)
R46 = tform2rotm(T46)
RPY46 = rotm2eul(R46)
aa46 = rotm2axang(R46)

%% LINK5 - FLANGE

t56 = tform2trvec(T56)
R56 = tform2rotm(T56)
RPY56 = rotm2eul(R56)
aa56 = rotm2axang(R56)


