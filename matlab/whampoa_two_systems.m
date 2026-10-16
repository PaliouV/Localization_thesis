% Τρόπος 1: περισσότεροι δορυφόροι με ένα «δεύτερο σύστημα» από το ίδιο GPS.
%
% Ο gnssMeasurementGenerator έχει μόνο GPS (~8 δορυφόροι πάνω από τη μάσκα 10°), ενώ ο
% δέκτης του [S4] μάλλον έβλεπε κι άλλα συστήματα (δεν αναφέρεται). Εδώ βάζουμε στο ίδιο
% όχημα ΔΥΟ γεννήτριες: μία με την κανονική ώρα και μία με ώρα μετατοπισμένη κατά OFFSET
% ώρες. Οι δορυφόροι της δεύτερης είναι σε άλλα σημεία του ουρανού, οπότε μαζί μοιάζουν με
% «GPS + ένα δεύτερο σύστημα». ΠΡΟΣΕΓΓΙΣΗ: δεν είναι πραγματικοί BeiDou / Galileo.
%
% Φρένο (συμφωνία 6 Οκτ 2026): αν με 2 συστήματα το λάθος δεν πλησιάζει το [S4]
% (μέσο 31 m, χωρίς θέση 0%), σταματάμε με το MATLAB.
%
% Θέση με lsPosition (όχι receiverposition, βλ. lsPosition.m), δύο τρόποι:
%   όλα σήματα   — όλα, και οι ανακλάσεις
%   1 / δορυφόρο — το απευθείας αν υπάρχει, αλλιώς η πιο σύντομη ανάκλαση
%
% Τρέχει:  matlab -batch "run('matlab/whampoa_two_systems.m')"
% Γράφει:  matlab/whampoa_two_systems.csv

here = fileparts(mfilename("fullpath"));

REF = [22.3047 114.1902 0];
DATE = datetime(2021,6,15);                 % ΠΑΡΑΔΟΧΗ (άγνωστη στο [S4])
START_HOURS = [0 4 8 12 16 20];
SPEED = 8;
LAPS = 3;
ANTENNA_HEIGHT = 1.5;
OFFSETS = [4 8];                            % ώρες μετατόπισης του «δεύτερου συστήματος»

L = readtable(fullfile(here, "whampoa_loops.csv"), TextType="string");
names = unique(L.loop, "stable");

B = readgeotable(fullfile(here, "whampoa.osm"), Layer="buildings");
[V, F] = buildingMesh(B, REF);
lo = min(V); hi = max(V); center = [(lo(1:2) + hi(1:2)) / 2, 0];
buildingsMesh = extendedObjectMesh(V - center, F);

% offset, loop, hour, epoch, συστήματα (1/2), τρόπος (1 όλα / 2 ένα ανά δορ.), err_2d, along, cross, n_sats
rows = [];
for off = OFFSETS
    for k = 1:numel(names)
        pts = [L.lat(L.loop == names(k)) L.lon(L.loop == names(k))];
        wp = lla2enu([pts zeros(size(pts,1),1)], REF, "ellipsoid");
        wp = [repmat(wp, LAPS, 1); wp(1,:)];
        wp(:,3) = 0;
        wp([false; vecnorm(diff(wp), 2, 2) < 0.5], :) = [];
        for h = START_HOURS
            scenario = drivingScenario(SampleTime=1, GeoReference=REF);
            actor(scenario, Position=center, Length=hi(1)-lo(1), Width=hi(2)-lo(2), ...
                  Height=hi(3), Mesh=buildingsMesh);
            ego = vehicle(scenario, ClassID=1);
            trajectory(ego, wp, SPEED);
            rng(h)
            t0 = DATE + hours(h) - hours(8);
            g1 = gnssMeasurementGenerator(SampleRate=1, ReferenceLocation=REF, InitialTime=t0);
            g2 = gnssMeasurementGenerator(SampleRate=1, ReferenceLocation=REF, InitialTime=t0 + hours(off));
            addSensors(scenario, {g1, g2}, ego.ActorID, [0 0 ANTENNA_HEIGHT]);

            n = 0;
            while advance(scenario)
                n = n + 1;
                [p1, s1, st1] = g1();
                [p2, s2, st2] = g2();
                truth = ego.Position + [0 0 ANTENNA_HEIGHT];
                yaw = deg2rad(ego.Yaw);
                sets = {{p1, s1, st1.LOS}, {[p1; p2], [s1; s2], [st1.LOS; st2.LOS]}};
                for sys = 1:2
                    [p, satPos, los] = sets{sys}{:};
                    for m = 1:2
                        sel = true(size(p));
                        if m == 2, sel = onePerSatellite(p, satPos, los); end
                        [e, a, c, ns] = solve(p(sel), satPos(sel,:), REF, truth, yaw);
                        rows(end+1, :) = [off, k, h, n, sys, m, e, a, c, ns]; %#ok<AGROW>
                    end
                end
            end
            fprintf("μετατόπιση %d h, βρόχος %s, %02d:00 έτοιμος\n", off, names(k), h);
        end
    end
end

T = array2table(rows, VariableNames=["offset","loop","hour","epoch","systems","mode", ...
                                     "err_2d","err_along","err_cross","n_sats"]);
writetable(T, fullfile(here, "whampoa_two_systems.csv"));

MODES = ["όλα σήματα", "1 / δορυφόρο"];
fprintf("\nΌλοι οι βρόχοι και οι ώρες μαζί:\n");
fprintf("  %-22s %-13s %8s %8s %8s %8s %8s %8s %7s\n", "", "τρόπος", "χ.θέση", "μέσο", "διάμεσο", "STD", "P95", "max", "δορυφ.");
for off = OFFSETS
    for sys = 1:2
        for m = 1:2
            if sys == 1 && off ~= OFFSETS(1), continue, end      % 1 σύστημα: ίδιο για κάθε μετατόπιση
            r = T(T.offset == off & T.systems == sys & T.mode == m, :);
            e = r.err_2d(~isnan(r.err_2d));
            if sys == 1, label = "1 σύστημα (GPS)"; else, label = sprintf("2 συστήματα (+%d h)", off); end
            fprintf("  %-22s %-13s %7.0f%% %6.1f m %6.1f m %6.1f m %6.1f m %6.0f m %7.1f\n", label, MODES(m), ...
                    100*mean(isnan(r.err_2d)), mean(e), median(e), std(e), prctile(e, 95), max(e), mean(r.n_sats));
        end
    end
end
fprintf("  [S4] canyon 1 (πραγματικό)          χ.θέση 0%%, μέσο 31.0, STD 37.7, max 177.6 m\n");


function sel = onePerSatellite(p, satPos, los)
% Ένα σήμα ανά δορυφόρο: το απευθείας αν υπάρχει, αλλιώς η πιο σύντομη ανάκλαση.
    sel = false(size(p));
    [~, ~, id] = unique(round(satPos), "rows");
    for s = unique(id)'
        idx = find(id == s);
        if any(los(idx)), idx = idx(los(idx)); end
        [~, j] = min(p(idx));
        sel(idx(j)) = true;
    end
end

function [e, a, c, ns] = solve(p, satPos, ref, truth, yaw)
% 2D λάθος, κατά μήκος, εγκάρσια· NaN αν δεν υπάρχουν τουλάχιστον 4 διαφορετικοί δορυφόροι.
    ns = size(unique(round(satPos), "rows"), 1);
    if ns < 4
        e = NaN; a = NaN; c = NaN;
        return
    end
    est = lsPosition(p, satPos, ref);
    dx = est(1) - truth(1);  dy = est(2) - truth(2);
    e = hypot(dx, dy);
    a = dx*cos(yaw) + dy*sin(yaw);
    c = -dx*sin(yaw) + dy*cos(yaw);
end
