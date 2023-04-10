mwLS = [42.3013 -71.375 0];
latlim = [mwLS(1)-0.003 mwLS(1)+0.003];
lonlim = [mwLS(2)-0.003 mwLS(2)+0.003];
fig = figure;
g = geoaxes(fig,Basemap="satellite");
geolimits(latlim,lonlim)
pl1lat = [42.3028 42.30325 42.3027 42.3017 42.3019]';
pl1lon = [-71.37527 -71.37442 -71.3736 -71.37378 -71.375234]';
pl1Poly = [pl1lat pl1lon];
pl2lat = [42.30035 42.2999 42.2996 42.2999]';
pl2lon = [-71.3762 -71.3734 -71.37376 -71.37589]';
pl2poly = [pl2lat pl2lon];
cs = uavCoverageSpace(Polygons={pl1Poly,pl2poly},UseLocalCoordinates=false,ReferenceLocation=mwLS);
ReferenceHeight = 25;
cs.UnitWidth = 20;
show(cs,Parent=g);
setCoveragePattern(cs,1,SweepAngle=85)
setCoveragePattern(cs,2,SweepAngle=5)
cp = uavCoveragePlanner(cs,Solver="Exhaustive");
takeoff = [42.30089 -71.3752, 0];
[wp,soln] = plan(cp,takeoff);
hold on
geoplot(wp(:,1),wp(:,2),LineWidth=1.5);
geoplot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")
legend("","","Path","Takeoff/Landing")
hold off
area = [5 8.75; 5 27.5; 17.5 22.5; 25 31.25; 35 31.25; 30 20; 15 6.25];
polygons = coverageDecomposition(area);
cs = uavCoverageSpace(Polygons=polygons);
cpeExh = uavCoveragePlanner(cs,Solver="Exhaustive");
cpMin = uavCoveragePlanner(cs,Solver="MinTraversal");
cpeExh.SolverParameters.VisitingSequence = [2 1];
cpMin.SolverParameters.VisitingSequence = [2 1];
[wptsExh,solnExh] = plan(cpeExh,takeoff,landing);
[wptsMin,solnMin] = plan(cpMin,takeoff,landing);
figure
show(cs);
title("Exhaustive Solver Algorithm")
exampleHelperPlotTakeoffLandingLegend(takeoff,landing,wptsExh)
figure
show(cs);
title("Minimum Traversal Solver Algorithm")
exampleHelperPlotTakeoffLandingLegend(takeoff,landing,wptsMin)
exportWaypointsPlan(cpMin,solnMin,"coveragepath.waypoints",ReferenceFrame="NED")
coverageWidth = 65;
region = [-210 130; 10 130; 10 20; 110 20;
          110 -130; -140 -130; -140 -20; -210 -20];
takeoff = [-250 150 0];
landing = [0 -200 0];
uavElevation = 150;
geocenter = [-45 71 0];
cs = uavCoverageSpace(Polygons=region, ...
                      UnitWidth=coverageWidth, ...
                      ReferenceHeight=uavElevation, ...
                      ReferenceLocation=geocenter);

cs.show;
title("Coverage Space")
cp = uavCoveragePlanner(cs);
[waypoints,solnInfo] = cp.plan(takeoff,landing);
hold on
plot(waypoints(:,1),waypoints(:,2))
scatter(takeoff(1),takeoff(2),"filled")
scatter(landing(1),landing(2),"filled")
legend("","Path","Takeoff","Landing")
hold off
exportWaypointsPlan(cp,solnInfo,"customCoverage.waypoints");
mission = uavMission(PlanFile="customCoverage.waypoints",Speed=10,InitialYaw=90);
exampleHelperSimulateUAVMission(mission,geocenter)



