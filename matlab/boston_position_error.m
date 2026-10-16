% Λάθος θέσης GPS στο παράδειγμα της MathWorks (κέντρο Βοστώνης).
%
% Το παράδειγμα "Simulate GNSS Multipath Effects in Urban Canyon Environment"
% βγάζει μόνο αποστάσεις προς τους δορυφόρους (pseudoranges) και αν κάθε σήμα
% είναι απευθείας (LOS) ή ανάκλαση. ΔΕΝ υπολογίζει θέση. Εδώ προσθέτουμε:
%   1. θέση από τις αποστάσεις, με απλά ελάχιστα τετράγωνα (lsPosition.m)
%   2. σύγκριση με την πραγματική θέση του οχήματος → λάθος θέσης
%
% Σκοπός: έλεγχος αν η προσομοίωση δίνει λογικά νούμερα (π.χ. δεκάδες μέτρα,
% όχι 1 ή 1000) πριν τη χρησιμοποιήσουμε για το Χονγκ Κονγκ.
%
% Χρειάζεται το αρχείο του παραδείγματος της MathWorks στον ίδιο φάκελο (git-ignored):
%   downtownBostonDrivingScenario.mat
% Τρέχει:  matlab -batch "run('matlab/boston_position_error.m')"
% Γράφει:  matlab/boston_errors.csv

here = fileparts(mfilename("fullpath"));
load(fullfile(here, "downtownBostonDrivingScenario.mat"), "scenario", "egoVehicle");

% --- Ίδιες ρυθμίσεις με το παράδειγμα της MathWorks ---
rng default
t0 = datetime(2022,12,8,10,0,0);
gnss = gnssMeasurementGenerator(SampleRate=1/scenario.SampleTime, ...
                                ReferenceLocation=scenario.GeoReference, ...
                                InitialTime=t0);
mountingLocation = [0 0 1.5];
addSensors(scenario, {gnss}, egoVehicle.ActorID, mountingLocation);

% --- Οδήγηση: σε κάθε βήμα, θέση από το GPS και λάθος ---
rows = [];
while advance(scenario)
    [p, satPos, status] = gnss();
    if numel(p) < 4                     % με λιγότερους από 4 δεν βγαίνει θέση
        continue
    end
    [~, ~, hdop] = receiverposition(p, satPos);                     % μόνο για το HDOP
    est = lsPosition(p, satPos, gnss.ReferenceLocation);             % όχι receiverposition: βλ. lsPosition.m
    truth = egoVehicle.Position + mountingLocation;

    dx = est(1) - truth(1);
    dy = est(2) - truth(2);
    yaw = deg2rad(egoVehicle.Yaw);
    along = dx*cos(yaw) + dy*sin(yaw);        % μπρος (+) / πίσω (−)
    cross = -dx*sin(yaw) + dy*cos(yaw);       % στο πλάι

    rows(end+1, :) = [scenario.SimulationTime, truth(1:2), est(1:2), ...
                      hypot(dx, dy), along, cross, sum(status.LOS), sum(~status.LOS), hdop]; %#ok<AGROW>
end

T = array2table(rows, VariableNames=["time","true_x","true_y","est_x","est_y", ...
                                     "err_2d","err_along","err_cross","n_los","n_reflected","hdop"]);
writetable(T, fullfile(here, "boston_errors.csv"));

e = T.err_2d;
fprintf("Μετρήσεις: %d, διάρκεια %.1f s\n", height(T), T.time(end) - T.time(1));
fprintf("Λάθος 2D:  μέσο %.1f m, STD %.1f m, RMS %.1f m, μέγιστο %.1f m\n", ...
        mean(e), std(e), sqrt(mean(e.^2)), max(e));
fprintf("Μέσο |εγκάρσιο| / μέσο |κατά μήκος| = %.2f\n", mean(abs(T.err_cross)) / mean(abs(T.err_along)));
fprintf("Σήματα ανά μέτρηση: απευθείας (LOS) %.1f, ανακλάσεις %.1f (μέσος όρος)\n", ...
        mean(T.n_los), mean(T.n_reflected));
fprintf("Αποθηκεύτηκε: matlab/boston_errors.csv\n");
