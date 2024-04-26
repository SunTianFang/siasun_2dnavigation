// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Abstract interface for objects solving linear systems of various
// kinds.

#ifndef CERES_INTERNAL_LINEAR_SOLVER_H_
#define CERES_INTERNAL_LINEAR_SOLVER_H_

#include <cstddef>
#include <map>
#include <string>
#include <vector>


namespace myceres {
namespace internal {

enum LinearSolverTerminationType {
  // Termination criterion was met.
  LINEAR_SOLVER_SUCCESS,

  // Solver ran for max_num_iterations and terminated before the
  // termination tolerance could be satisfied.
  LINEAR_SOLVER_NO_CONVERGENCE,

  // Solver was terminated due to numerical problems, generally due to
  // the linear system being poorly conditioned.
  LINEAR_SOLVER_FAILURE,

  // Solver failed with a fatal error that cannot be recovered from,
  // e.g. CHOLMOD ran out of memory when computing the symbolic or
  // numeric factorization or an underlying library was called with
  // the wrong arguments.
  LINEAR_SOLVER_FATAL_ERROR
};

// This enum controls the fill-reducing ordering a sparse linear
// algebra library should use before computing a sparse factorization
// (usually Cholesky).
enum OrderingType {
  NATURAL, // Do not re-order the matrix. This is useful when the
           // matrix has been ordered using a fill-reducing ordering
           // already.
  AMD      // Use the Approximate Minimum Degree algorithm to re-order
           // the matrix.
};

}  // namespace internal
}  // namespace ceres

#endif  // CERES_INTERNAL_LINEAR_SOLVER_H_
