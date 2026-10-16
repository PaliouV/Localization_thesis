function [V, F] = buildingMesh(B, ref)
% Κάθε κτίριο: πρίσμα από MinHeight έως MaxHeight πάνω στο περίγραμμά του
% (καπάκι πάνω/κάτω + τοίχοι). Όλα μαζί σε ένα πλέγμα, σε μέτρα ENU γύρω από το ref.
    V = zeros(0,3);  F = zeros(0,3);
    G = geotable2table(B, ["Lat","Lon"]);
    for i = 1:height(G)
        lat = G.Lat{i}(:);  lon = G.Lon{i}(:);
        zb = B.MinHeight(i);  zt = B.MaxHeight(i);
        if zt <= zb, continue, end
        ok = ~isnan(lat);
        xy = nan(numel(lat), 2);
        xy(ok,:) = lla2enu([lat(ok) lon(ok) zeros(nnz(ok),1)], ref, "ellipsoid") * [1 0; 0 1; 0 0];
        warning("off", "MATLAB:polyshape:repairedBySimplify");
        ps = polyshape(xy(:,1), xy(:,2));
        if ps.NumRegions == 0, continue, end
        tri = triangulation(ps);
        P = tri.Points;  C = tri.ConnectivityList;  n = size(P,1);
        base = size(V,1);
        V = [V; P zb*ones(n,1); P zt*ones(n,1)]; %#ok<AGROW>
        F = [F; base + C(:,[1 3 2]); base + n + C]; %#ok<AGROW>      % κάτω / πάνω καπάκι
        for k = 1:ps.NumRegions + ps.NumHoles                          % τοίχοι κάθε περιγράμματος
            [bx, by] = boundary(ps, k);
            m = numel(bx) - 1;                                         % το boundary κλείνει στην αρχή
            b0 = size(V,1);
            V = [V; bx(1:m) by(1:m) zb*ones(m,1); bx(1:m) by(1:m) zt*ones(m,1)]; %#ok<AGROW>
            j = (1:m)';  jn = mod(j, m) + 1;
            F = [F; b0 + [j jn m+jn]; b0 + [j m+jn m+j]]; %#ok<AGROW>
        end
    end
end
