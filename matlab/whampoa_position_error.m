% Λάθος θέσης GPS στο Whampoa (Hung Hom, Χονγκ Κονγκ), με τα κτίρια του OpenStreetMap.
%
% Σκοπός: έλεγχος αν η προσομοίωση του MATLAB δίνει λάθος θέσης κοντά στις
% ΠΡΑΓΜΑΤΙΚΕΣ μετρήσεις του [S4] Wen & Hsu, urban canyon 1 (Whampoa — από το Σχ. 7,
% επιβεβαιωμένο από τον χρήστη 5 Οκτ 2026): μέσο 31.02 m, STD 37.69 m, μέγιστο 177.59 m.
%
% Ο ακριβής βρόχος του [S4] δεν είναι γνωστός. Γι' αυτό τρέχουμε 6 βρόχους μέσα στο
% Whampoa Garden, σε 6 ώρες της ημέρας (η θέση των δορυφόρων αλλάζει), για να φανεί
% πόσο αλλάζει το λάθος από βρόχο σε βρόχο και από ώρα σε ώρα.
%
% Βήματα:
%   1. κτίρια από το whampoa.osm (readgeotable, Mapping Toolbox) → ένα πλέγμα τριγώνων
%      (κάθε περίγραμμα «σηκώνεται» σε πρίσμα με το ύψος του), όπως στο παράδειγμα της Βοστώνης
%   2. όχημα που κάνει γύρους σε κάθε βρόχο, πάνω στη γεωμετρία των δρόμων του OSM
%   3. gnssMeasurementGenerator (ray tracing στα κτίρια) → lsPosition → λάθος θέσης
%
% Παραδοχές / απλοποιήσεις (να δηλωθούν):
%   - ύψη κτιρίων όπως τα βγάζει το readgeotable: «height» του OSM, αλλιώς όροφοι × 3 m,
%     αλλιώς 9 m (62 από τα 405 κτίρια)
%   - το όχημα κινείται πάνω στη γραμμή του δρόμου του OSM (κέντρο οδοστρώματος, όχι λωρίδα)
%   - θέση με απλά ελάχιστα τετράγωνα (lsPosition) πάνω σε ΟΛΑ τα σήματα (και ανακλάσεις). Το [S4]
%     μέτρησε τη λύση του ίδιου του δέκτη u-blox (NMEA), που έχει δικό της αλγόριθμο
%   - ημερομηνία: η ημερομηνία του πειράματος του [S4] δεν αναφέρεται· διαλέξαμε μία
%
% Τρέχει:  matlab -batch "run('matlab/whampoa_position_error.m')"
% Γράφει:  matlab/whampoa_errors.csv (κάθε μέτρηση), matlab/whampoa_summary.csv

here = fileparts(mfilename("fullpath"));

% --- Ρυθμίσεις ---
REF = [22.3047 114.1902 0];                 % κέντρο της σκηνής (lat, lon, alt)
DATE = datetime(2021,6,15);                 % ΠΑΡΑΔΟΧΗ: ημερομηνία (άγνωστη στο [S4])
START_HOURS = [0 4 8 12 16 20];             % τοπική ώρα Χονγκ Κονγκ (UTC+8)
SPEED = 8;                                  % m/s
LAPS = 3;
ANTENNA_HEIGHT = 1.5;                       % m, όπως στο παράδειγμα της MathWorks

% Βρόχοι: γύρω από τα οικοδομικά τετράγωνα, ακολουθώντας τους δρόμους του OSM
% (φτιάχνονται από το whampoa_loops.py → whampoa_loops.csv: loop, lat, lon)
L = readtable(fullfile(here, "whampoa_loops.csv"), TextType="string");
names = unique(L.loop, "stable");
LOOPS = struct("name", cellstr(names)', "desc", "", "pts", []);
for k = 1:numel(names)
    LOOPS(k).pts = [L.lat(L.loop == names(k)) L.lon(L.loop == names(k))];
    LOOPS(k).desc = sprintf("%d σημεία", size(LOOPS(k).pts, 1));
end

% --- 1. Κτίρια → ένα πλέγμα τριγώνων ---
B = readgeotable(fullfile(here, "whampoa.osm"), Layer="buildings");
[V, F] = buildingMesh(B, REF);
fprintf("Κτίρια: %d → πλέγμα %d κορυφές, %d τρίγωνα\n", height(B), size(V,1), size(F,1));
lo = min(V); hi = max(V); center = [(lo(1:2) + hi(1:2)) / 2, 0];
buildingsMesh = extendedObjectMesh(V - center, F);

% --- 2–3. Κάθε βρόχος × κάθε ώρα ---
rows = [];
summary = [];
for L = 1:numel(LOOPS)
    loop = LOOPS(L);
    wp = lla2enu([loop.pts zeros(size(loop.pts,1),1)], REF, "ellipsoid");
    wp = [repmat(wp, LAPS, 1); wp(1,:)];             % LAPS γύροι, κλείνει στην αρχή
    wp(:,3) = 0;
    wp([false; vecnorm(diff(wp), 2, 2) < 0.5], :) = [];   % όχι διπλά διαδοχικά σημεία
    for h = START_HOURS
        scenario = drivingScenario(SampleTime=1, GeoReference=REF);
        actor(scenario, Position=center, Length=hi(1)-lo(1), Width=hi(2)-lo(2), ...
              Height=hi(3), Mesh=buildingsMesh);
        ego = vehicle(scenario, ClassID=1);
        trajectory(ego, wp, SPEED);

        rng(h)                                       % επαναλήψιμο ανά ώρα
        t0 = DATE + hours(h) - hours(8);             % ώρα Χονγκ Κονγκ → UTC
        gnss = gnssMeasurementGenerator(SampleRate=1, ReferenceLocation=REF, InitialTime=t0);
        addSensors(scenario, {gnss}, ego.ActorID, [0 0 ANTENNA_HEIGHT]);

        nEpochs = 0; nNoFix = 0; err = [];
        while advance(scenario)
            nEpochs = nEpochs + 1;
            [p, satPos, status] = gnss();
            % Θέση μόνο με τουλάχιστον 4 ΔΙΑΦΟΡΕΤΙΚΟΥΣ δορυφόρους. Οι ανακλάσεις ενός
            % δορυφόρου έρχονται ως επιπλέον γραμμές με την ίδια θέση δορυφόρου· με λιγότερους
            % από 4 διαφορετικούς οι εξισώσεις δεν λύνονται (λύσεις χιλιομέτρων / NaN).
            % Ένας δέκτης τότε δεν δίνει θέση.
            if size(unique(round(satPos), "rows"), 1) < 4
                nNoFix = nNoFix + 1;
                continue
            end
            est = lsPosition(p, satPos, REF);           % όχι receiverposition: βλ. lsPosition.m
            truth = ego.Position + [0 0 ANTENNA_HEIGHT];
            dx = est(1) - truth(1);  dy = est(2) - truth(2);
            yaw = deg2rad(ego.Yaw);
            along = dx*cos(yaw) + dy*sin(yaw);
            cross = -dx*sin(yaw) + dy*cos(yaw);
            e2d = hypot(dx, dy);
            err(end+1) = e2d; %#ok<AGROW>
            rows(end+1, :) = [L, h, scenario.SimulationTime, truth(1:2), est(1:2), e2d, along, cross, ...
                              sum(status.LOS), sum(~status.LOS)]; %#ok<AGROW>
        end
        summary(end+1, :) = [L, h, nEpochs, 100*nNoFix/max(nEpochs,1), mean(err), std(err), ...
                             sqrt(mean(err.^2)), max(err)]; %#ok<AGROW>
        fprintf("Βρόχος %s, %02d:00 — μετρήσεις %d, χωρίς θέση %.0f%%, μέσο %.1f, STD %.1f, RMS %.1f, max %.1f m\n", ...
                loop.name, h, summary(end,3:8));
    end
end

T = array2table(rows, VariableNames=["loop","hour","time","true_x","true_y","est_x","est_y", ...
                                     "err_2d","err_along","err_cross","n_los","n_reflected"]);
writetable(T, fullfile(here, "whampoa_errors.csv"));
S = array2table(summary, VariableNames=["loop","hour","epochs","no_fix_pct","mean","std","rms","max"]);
writetable(S, fullfile(here, "whampoa_summary.csv"));

fprintf("\nΑνά βρόχο (όλες οι ώρες μαζί):\n");
for L = 1:numel(LOOPS)
    e = T.err_2d(T.loop == L);
    fprintf("  %s  %-12s μέσο %5.1f  STD %5.1f  RMS %5.1f  max %6.1f m  |εγκ.|/|κ.μ.| = %.2f\n", ...
            LOOPS(L).name, LOOPS(L).desc, mean(e), std(e), sqrt(mean(e.^2)), max(e), ...
            mean(abs(T.err_cross(T.loop == L))) / mean(abs(T.err_along(T.loop == L))));
end
fprintf("  [S4] canyon 1 (πραγματικό)    μέσο  31.0  STD  37.7  RMS ~48.8  max  177.6 m\n");
