% run your Simulink model
% simOut = sim("satellite_dynamics");
simOut= out;

posTS = simOut.logsout.get("X_icrf").Values;
velTS = simOut.logsout.get("V_icrf").Values;

attTS = simOut.logsout.get("q_icrf2b_star_tracker").Values;

attEmpty = simOut.logsout.get("q_icrf2b").Values;

attEmpty.Data = interp1(attTS.Time,attTS.Data,velTS.Time);
attEmpty.Time = velTS.Time;
attTS = attEmpty;


% attEmpty = simOut.logsout.get("q_icrf2b").Values;
% 
% data_size = size(attTS.Data);
% 
% attEmpty.Data(1:data_size(1),1:data_size(2)) = attTS.Data;
% 
% attTS = attEmpty;

% attTS = simOut.logsout.get("q_icrf2b").Values;

% Convert to timetables
posTT = timeseries2timetable(posTS);
velTT = timeseries2timetable(velTS);
attTT = timeseries2timetable(attTS);



% Keep only unique time stamps
[tu, ia] = unique(posTT.Time, "stable");
posTT = posTT(ia,:);
velTT = velTT(ia,:);
attTT = attTT(ia,:);

% Optional: resample to a regular grid, e.g. 1 second
posTT = retime(posTT, "regular", "linear", "TimeStep", seconds(1));
velTT = retime(velTT, "regular", "linear", "TimeStep", seconds(1));
attTT = retime(attTT, "regular", "linear", "TimeStep", seconds(1));

startDate = datetime(2028,1,1,0,0,0,"TimeZone","UTC");
stopDate  = startDate + posTT.Time(end);

sc = satelliteScenario(startDate, stopDate, 1);

sat = satellite(sc, posTT, velTT, ...
    "CoordinateFrame","inertial", ...
    "Name","SC");

pointAt(sat, attTT, ...
    CoordinateFrame="inertial", ...
    Format="quaternion", ...
    ExtrapolationMethod="fixed");

viewer = satelliteScenarioViewer(sc);
sat.Visual3DModel = "SmallSat.glb";   % optional
coordinateAxes(sat, Scale=2);         % optional
camtarget(viewer, sat);               % optional

play(sc)