function enu = lsPosition(p, satPos, ref)
% Θέση δέκτη από pseudoranges με απλά ελάχιστα τετράγωνα (άγνωστοι: x, y, z ECEF + ρολόι).
% Επιστρέφει θέση σε μέτρα ENU γύρω από το ref.
%
% Γιατί όχι receiverposition: τρέχοντας τη σκηνή χωρίς κτίρια, οι pseudoranges του
% gnssMeasurementGenerator απέχουν λίγα εκατοστά από τις γεωμετρικές αποστάσεις προς την
% αληθινή θέση, αλλά η receiverposition έδινε θέση ~33 m δυτικά, σταθερά (whampoa_sanity.m,
% 6 Οκτ 2026). Πιθανή αιτία (δεν επαληθεύτηκε στην τεκμηρίωση): η receiverposition διορθώνει
% την περιστροφή της Γης ενώ ο προσομοιωτής δεν τη βάζει. Εδώ λύνουμε το ίδιο μοντέλο που
% χρησιμοποιεί ο προσομοιωτής: pseudorange = απόσταση + ρολόι.
    [x0, y0, z0] = geodetic2ecef(wgs84Ellipsoid, ref(1), ref(2), ref(3));
    s = [x0 y0 z0 0];                                % αρχή: το κέντρο της σκηνής, ρολόι 0
    for it = 1:10
        d = satPos - s(1:3);
        r = vecnorm(d, 2, 2);
        H = [-d ./ r, ones(size(r))];
        ds = H \ (p(:) - (r + s(4)));
        s = s + ds';
        if norm(ds) < 1e-4, break, end
    end
    [e, n, u] = ecef2enu(s(1), s(2), s(3), ref(1), ref(2), ref(3), wgs84Ellipsoid);
    enu = [e n u];
end
