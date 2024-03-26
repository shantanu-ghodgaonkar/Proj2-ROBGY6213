function avgMatrix = leastSquaresAvg(A, B)

  % Check matrix dimensions
  if ~isequal(size(A), size(B))
    error('Matrices A and B must have the same dimensions (3x3)');
  end

  % Calculate means of each matrix
  meanA = mean(A(:));
  meanB = mean(B(:));

  % Calculate squared differences using vectorization
  squaredDiffA = (A - meanA).^2;
  squaredDiffB = (B - meanB).^2;

  % Calculate total sum of squared differences for each matrix
  sumSqDiffA = sum(squaredDiffA(:));
  sumSqDiffB = sum(squaredDiffB(:));

  % Prevent division by zero (if both matrices have same elements)
  if sumSqDiffA + sumSqDiffB == 0
    warning('Matrices A and B have identical elements. Least squares average is the same as element-wise average.');
    avgMatrix = (A + B) / 2;
    return;
  end

  % Calculate weights based on sum of squared differences
  weightA = sumSqDiffB / (sumSqDiffA + sumSqDiffB);
  weightB = sumSqDiffA / (sumSqDiffA + sumSqDiffB);

  % Least squares average using weights and original matrices
  avgMatrix = weightA * A + weightB * B;
end
