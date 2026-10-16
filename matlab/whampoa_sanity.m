% Έλεγχος ότι το whampoa_position_error.m δεν βγάζει μεγάλα λάθη από δικό μας λάθος.
%
% Ίδιοι βρόχοι, ίδιες ώρες, ίδιες ρυθμίσεις. Τέσσερις τρόποι:
%   όλα σήματα    — όπως το whampoa_position_error.m: όλα τα σήματα (και ανακλάσεις)
%   1 / δορυφόρο  — ένα σήμα ανά δορυφόρο: το απευθείας αν υπάρχει, αλλιώς η πιο σύντομη
%                   ανάκλαση (ο δέκτης κλειδώνει ένα σήμα ανά δορυφόρο)
%   μόνο LOS      — Τεστ 2: μόνο τα απευθείας σήματα, οι ανακλάσεις πετιούνται
%   χωρίς κτίρια  — Τεστ 1: η ίδια σκηνή χωρίς κανένα κτίριο
% Οι τρεις πρώτοι χρησιμοποιούν τις ΙΔΙΕΣ μετρήσεις (ένα ray tracing), αλλάζει μόνο ποια
% σήματα μπαίνουν στον υπολογισμό θέσης.
%
% Αναμενόμενο αν ο κώδικας είναι σωστός: «χωρίς κτίρια» λίγα μέτρα (RangeAccuracy 1 m),
% «μόνο LOS» μικρό λάθος όπου υπάρχει θέση (λιγότεροι δορυφόροι → λιγότερη διαθεσιμότητα).
%
% Τρέχει:  matlab -batch "run('matlab/whampoa_sanity.m')"
% Γράφει:  matlab/whampoa_sanity.csv

here = fileparts(mfilename("fullpath"));

REF = [22.3047 114.1902 0];
DATE = datetime(2021,6,15);
START_HOURS = [0 4 8 12 16 20];
SPEED = 8;
LAPS = 3;
ANTENNA_HEIGHT = 1.5;
MODES = ["όλα σήματα", "1 / δορυφόρο", "μόνο LOS", "χωρίς κτίρια"];

L = readtable(fullfile(here, "whampoa_loops.csv"), TextType="string");
names = unique(L.loop, "stable");

B = readgeotable(fullfile(here, "whampoa.osm"), Layer="buildings");
[V, F] = buildingMesh(B, REF);
lo = min(V); hi = max(V); center = [(lo(1:2) + hi(1:2)) / 2, 0];
buildingsMesh = extendedObjectMesh(V - center, F);

rows = [];   % loop, hour, mode, epoch, err_2d (NaN = χωρίς θέση), n_signals, n_sats
for k = 1:numel(names)
    pts = [L.lat(L.loop == names(k)) L.lon(L.loop == names(k))];
    wp = lla2enu([pts zeros(size(pts,1),1)], REF, "ellipsoid");
    wp = [repmat(wp, LAPS, 1); wp(1,:)];
    wp(:,3) = 0;
    wp([false; vecnorm(diff(wp), 2, 2) < 0.5], :) = [];
    for h = START_HOURS
        for withBuildings = [true false]
            scenario = drivingScenario(SampleTime=1, GeoReference=REF);
            if withBuildings
                actor(scenario, Position=center, Length=hi(1)-lo(1), Width=hi(2)-lo(2), ...
                      Height=hi(3), Mesh=buildingsMesh);
            end
            ego = vehicle(scenario, ClassID=1);
            trajectory(ego, wp, SPEED);
            rng(h)
            gnss = gnssMeasurementGenerator(SampleRate=1, ReferenceLocation=REF, ...
                                            InitialTime=DATE + hours(h) - hours(8));
            addSensors(scenario, {gnss}, ego.ActorID, [0 0 ANTENNA_HEIGHT]);

            n = 0;
            while advance(scenario)
                n = n + 1;
                [p, satPos, status] = gnss();
                truth = ego.Position + [0 0 ANTENNA_HEIGHT];
                if withBuildings
                    modes = 1:3;
                else
                    modes = 4;
                end
                for m = modes
                    sel = pick(m, p, satPos, status.LOS);
                    e = solve(p(sel), satPos(sel,:), REF, truth);
                    rows(end+1, :) = [k, h, m, n, e, nnz(sel), ...
                                      size(unique(round(satPos(sel,:)), "rows"), 1)]; %#ok<AGROW>
                end
            end
        end
        fprintf("Βρόχος %s, %02d:00 έτοιμος\n", names(k), h);
    end
end

T = array2table(rows, VariableNames=["loop","hour","mode","epoch","err_2d","n_signals","n_sats"]);
writetable(T, fullfile(here, "whampoa_sanity.csv"));

fprintf("\nΌλοι οι βρόχοι και οι ώρες μαζί:\n");
fprintf("  %-14s %9s %8s %8s %8s %9s %10s %9s\n", "τρόπος", "χ. θέση", "μέσο", "διάμεσο", "P95", "max", "σήματα", "δορυφ.");
for m = 1:numel(MODES)
    r = T(T.mode == m, :);
    e = r.err_2d(~isnan(r.err_2d));
    fprintf("  %-14s %8.0f%% %7.1f m %7.1f m %7.1f m %8.0f m %10.1f %9.1f\n", MODES(m), ...
            100*mean(isnan(r.err_2d)), mean(e), median(e), prctile(e, 95), max(e), ...
            mean(r.n_signals), mean(r.n_sats));
end
fprintf("  [S4] canyon 1   χ. θέση 0%%, μέσο 31.0 m, max 177.6 m\n");


function sel = pick(mode, p, satPos, los)
% Ποια σήματα μπαίνουν στον υπολογισμό θέσης.
    switch mode
        case {1, 4}                       % όλα
            sel = true(size(p));
        case 3                            % μόνο απευθείας
            sel = los(:);
        case 2                            % ένα ανά δορυφόρο
            sel = false(size(p));
            [~, ~, id] = unique(round(satPos), "rows");
            for s = unique(id)'
                idx = find(id == s);
                if any(los(idx))
                    idx = idx(los(idx));
                end
                [~, j] = min(p(idx));     % η πιο σύντομη διαδρομή
                sel(idx(j)) = true;
            end
    end
end

function e = solve(p, satPos, ref, truth)
% 2D λάθος θέσης· NaN αν δεν υπάρχουν τουλάχιστον 4 διαφορετικοί δορυφόροι.
    if size(unique(round(satPos), "rows"), 1) < 4
        e = NaN;
        return
    end
    est = lsPosition(p, satPos, ref);
    e = hypot(est(1) - truth(1), est(2) - truth(2));
end
