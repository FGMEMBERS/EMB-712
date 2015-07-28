##
#  action-sim.nas   Updates various simulated features including:
#                    egt, fuel pressure, oil pressure, prop visibility, 
#                    stall warning, etc. every frame
##

#   Initialize local variables
var rpm = nil;
var fuel_pres = 0.0;
var oil_pres = 0.0;
var factor = nil;
var ias = nil;
var flaps = nil;
var gforce = nil;
var stall = nil;
var bsw = nil;
var node = nil;
var OnGround = nil;
var fuel_flow = nil;
var egt = nil;
var fuel_pump_volume = nil;

# set up filters for these actions

var fuel_pres_lowpass = aircraft.lowpass.new(0.5);
var oil_pres_lowpass = aircraft.lowpass.new(0.5);
var egt_lowpass = aircraft.lowpass.new(0.95);
var cdi0_lowpass = aircraft.lowpass.new(0.5);
var cdi1_lowpass = aircraft.lowpass.new(0.5);
var gs0_lowpass = aircraft.lowpass.new(0.5);
var gs1_lowpass = aircraft.lowpass.new(0.5);
var propGear0 = props.globals.getNode("gear/gear[0]", 1);
var theta0N = propGear0.getNode("theta0", 1);
var gear0Compression = propGear0.getNode("compression-m", 1);

var init_actions = func {
    theta0N.setDoubleValue(0.0);
    gear0Compression.setDoubleValue(0.0);
    setprop("engines/engine[0]/fuel-flow-gph", 0.0);
    setprop("/surface-positions/flap-pos-norm", 0.0);
    setprop("/instrumentation/airspeed-indicator/indicated-speed-kt", 0.0);
    setprop("/instrumentation/airspeed-indicator/pressure-alt-offset-deg", 0.0);
    setprop("/accelerations/pilot-g", 1.0);
    setprop("/controls/flight/aileron_in", 0.0);
    setprop("/controls/flight/elevator_in", 0.0);
    setprop("/instrumentation/nav[0]/filtered-cdiNAV0-deflection", 0.0);
    setprop("/instrumentation/nav[1]/filtered-cdiNAV1-deflection", 0.0);
    setprop("/instrumentation/nav[0]/filtered-gsNAV0-deflection", 0.0);
    setprop("/instrumentation/nav[1]/filtered-gsNAV1-deflection", 0.0);

    # Make sure that init_actions is called when the sim is reset
    setlistener("sim/signals/reset", init_actions); 

    # Request that the update fuction be called next frame
    settimer(update_actions, 0);
}

setprop("/instrumentation/nav[0]/gs-needle-deflection-norm",0);
setprop("/instrumentation/nav[1]/gs-needle-deflection-norm",0);

var update_actions = func {
##
#  This is a convenient cludge to model fuel pressure and oil pressure
##
    rpm = getprop("/engines/engine/rpm");
    if (rpm > 600.0) {
       fuel_pres = 6.8-3000/rpm;
       oil_pres = 62-12600/rpm;
    } else {
       fuel_pres = 0.0;
       oil_pres = 0.0;
    }

    if (getprop("/controls/engines/engine/fuel-pump")) {
    fuel_pres += 1.5;
    }

##
#  reduce fuel pump sound volume as rpm increases
##
   if (rpm < 1200) {
     fuel_pump_volume = 0.75/(0.002*rpm+1)
   } else {
     fuel_pump_volume = 0.0
   }

##
#  Save a factor used to make the prop disc disapear as rpm increases
##
    factor = 1.0 - rpm/2400;
    if ( factor < 0.0 ) {
        factor = 0.0;
    }

##
#  Stall Warning
##
    ias = getprop("/instrumentation/airspeed-indicator/indicated-speed-kt");
    flaps = getprop("/surface-positions/flap-pos-norm");
    gforce = getprop("/accelerations/pilot-g");
#    print("ias = ", ias, "  flaps = ", flaps);
#  pa28-161 Vs = 50 knots,  warn at 52
    stall = 50 - 7*flaps + 20*(gforce - 1.0);
    node = props.globals.getNode("/sim/alarms/stall-warning",1);
    if ( ias > stall ) {
      node.setBoolValue(0);
    }
    else {
      node.setBoolValue(1);
    }

##
#  Simulate egt from pilot's perspective using fuel flow and rpm
##
    fuel_flow = getprop("engines/engine[0]/fuel-flow-gph");
    egt = 325 - abs(fuel_flow - 10)*20;
    if (egt < 20) {egt = 20; }
    egt = egt*(rpm/2400)*(rpm/2400);

##
#  Compute the scissor link angles due to nose strut compression
##

    var theta0 = 0.0;
    # Compute the angle the nose gear scissor rotates due to nose gear strut compression

    H = 0.099543;  # Nose gear oleo strut extended length in m
    L = 0.060553;  # Nose gear scissor length in m
    phi = 0.964830;
    C = gear0Compression.getValue();
    if (C > 0.0) {
      theta0 = scissor_angle(H,C,L,phi);
    }

##
#  Disengage Joystick aileron if autopilot is controlling roll
##

  if ( getprop("autopilot/KAP140/locks/roll-axis")) { 
      aileron = getprop("controls/flight/AP_aileron");
  }
  else {
      aileron = getprop("controls/flight/aileron");
  }

##
#  Disengage Joystick elevator if autopilot is controlling pitch
##

  if ( getprop("autopilot/KAP140/locks/pitch-axis")) {
      elevator = getprop("controls/flight/AP_elevator");
  }
  else {
      elevator = getprop("controls/flight/elevator");
  }

  var cdiNAV0 = getprop("/instrumentation/nav[0]/heading-needle-deflection");
  var cdiNAV1 = getprop("/instrumentation/nav[1]/heading-needle-deflection");
  var gsNAV0  = getprop("/instrumentation/nav[0]/gs-needle-deflection-norm");
  var gsNAV1  = getprop("/instrumentation/nav[1]/gs-needle-deflection-norm");

    # outputs
    theta0N.setDoubleValue(theta0);
    setprop("/controls/flight/aileron_in", aileron);
    setprop("/controls/flight/elevator_in", elevator);
    setprop("/instrumentation/nav[0]/filtered-cdiNAV0-deflection", cdi0_lowpass.filter(cdiNAV0));
    setprop("/instrumentation/nav[1]/filtered-cdiNAV1-deflection", cdi1_lowpass.filter(cdiNAV1));
    setprop("/instrumentation/nav[0]/filtered-gsNAV0-deflection", gs0_lowpass.filter(gsNAV0));
    setprop("/instrumentation/nav[1]/filtered-gsNAV1-deflection", gs1_lowpass.filter(gsNAV1));
    setprop("/engines/engine[0]/egt-degf-fix", egt_lowpass.filter(egt));
    setprop("/sim/models/materials/propdisc/factor", factor);  
    setprop("/engines/engine/fuel-pressure-psi", fuel_pres_lowpass.filter(fuel_pres));
    setprop("/engines/engine/oil-pressure-psi", oil_pres_lowpass.filter(oil_pres));
    setprop("/sim/sound/fuel_pump_volume", fuel_pump_volume);

    settimer(update_actions, 0);
}

var scissor_angle = func(H,C,L,phi) {
    var a = (H - C)/2/L;
    # Use 2 iterates of Newton's method and 4th order Taylor series to 
    # approximate theta where sin(phi - theta) = a
    var theta = phi - 2*a/3 - a/3/(1-a*a/2);
    return theta;
}

# Setup listener call to start update loop once the fdm is initialized
# 
setlistener("/sim/signals/fdm-initialized", init_actions);  



