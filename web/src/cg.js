// Conjugate Gradient: solves A·x = b for a symmetric positive-definite A,
// without ever needing A itself — only a function that computes "A times
// this vector". That's what lets the same solver work whether the multiply
// comes from a sparse matrix (the next step) or, later, from a
// matrix-free computation that never assembles A at all.
//
// x0, if given, is the starting guess — later used for warm-starting one
// optimization iteration's solve from the previous iteration's answer,
// since density barely changes between iterations.
export function conjugateGradient(matVec, b, options = {}) {
  const { tolerance = 1e-8, maxIterations = b.length * 2, x0 } = options;

  let x = x0 ? x0.slice() : new Array(b.length).fill(0);
  let r = subtract(b, matVec(x));
  let p = r.slice();
  let rsOld = dot(r, r);

  const bNormSq = dot(b, b) || 1; // avoid divide-by-zero when b is all 0

  if (rsOld / bNormSq < tolerance * tolerance) {
    return { x, iterations: 0 };
  }

  for (let iter = 0; iter < maxIterations; iter++) {
    const Ap = matVec(p);
    const alpha = rsOld / dot(p, Ap);

    x = addScaled(x, p, alpha);
    r = addScaled(r, Ap, -alpha);

    const rsNew = dot(r, r);
    if (rsNew / bNormSq < tolerance * tolerance) {
      return { x, iterations: iter + 1 };
    }

    p = addScaled(r, p, rsNew / rsOld);
    rsOld = rsNew;
  }

  return { x, iterations: maxIterations };
}

function dot(a, b) {
  let sum = 0;
  for (let i = 0; i < a.length; i++) sum += a[i] * b[i];
  return sum;
}

function subtract(a, b) {
  return a.map((value, i) => value - b[i]);
}

// a + scalar * b
function addScaled(a, b, scalar) {
  return a.map((value, i) => value + scalar * b[i]);
}
