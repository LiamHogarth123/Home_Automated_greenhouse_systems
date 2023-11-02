function algebraicDist = collisionDetection(pointCloud, centerPoint)

% Check if any point in a matrix is within a specified radius from a center point.

% Calculate the Euclidean distance between the center point and each point in the matrix.
distances = sqrt(sum((pointCloud - centerPoint).^2, 2));

% Check if any of the distances are less than the specified radius.
algebraicDist = any(distances <= 1);

end