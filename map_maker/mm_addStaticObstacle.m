function map = mm_addStaticObstacle(map, x1, y1, x2, y2, envelope, costEnvelope)

    if nargin < 7
        costEnvelope = 0;
    end
    if nargin < 6
        envelope = 0;
    end
    
    % Add the static obstacle
    map = mm_addRectangle(map, x1, y1, x2, y2, 'STATIC_OBSTACLE');
    
    % Add the additional cost for the envelope
    if envelope > 0
        map = mm_addRectangle(map, ...
            x1 - envelope, y1 - envelope, ...
            x2 + envelope, y2 + envelope, ...
            costEnvelope);
    end
end