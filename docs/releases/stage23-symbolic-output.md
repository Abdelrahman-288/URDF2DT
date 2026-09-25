# Useful symbolic output

Generate Symbolic FK or Symbolic Jacobian on demand as before. The result selector
now includes Expanded equations (no aliases), alongside Matrix, Definitions and
Original full-precision text. Numerical J(q) remains available for Jacobians.

Click any matrix cell to inspect its selected entry, including constant entries.
Choose compact or expanded in the entry inspector. Copy entry copies its displayed
full-precision text; Copy expanded entry LaTeX copies the underlying entry formula.
The full matrix window provides the same controls.

Export publication LaTeX creates a standalone document, with a joint-coordinate
key and frame/model/source context. Choose compact ordered definitions or one
expanded equation per matrix entry. Compile using LuaLaTeX with fontspec, amsmath,
amssymb and breqn. breqn provides breakable displayed equations. Review typeset
line breaks for the chosen robot and journal template. No TeX engine is bundled;
the application exports source rather than a precompiled PDF.

## Mathematical and resource controls

- Compact representation uses the existing common-subexpression elimination.
- Expanded means original matrix expressions without t/M aliases. It is not a
  distributive expansion, trigonometric rewrite or global simplify operation.
- Small terms, source signs, offsets and stored geometry constants are preserved.
  No tolerance-based zeroing, geometry snapping or rational approximation is added.
- Expanded strings/LaTeX are produced in the existing isolated worker, cached with
  the model result and invalidated with it. Existing cancellation, 60-second timeout
  and 12-joint limits remain.
- Each expanded entry is limited to 20,000 operations; combined expanded text and
  LaTeX are capped at one million characters. Entries beyond the limit are explicitly
  unavailable. Compact output remains usable; expanded document export refuses a
  partial result rather than silently omitting equations.
- Native floating arithmetic and subsequent parsing can differ at floating-point
  roundoff. Tests compare the emitted original-expression strings exactly and
  compare reconstructed compact/expanded numerical values against the source model.

The existing raw text/LaTeX snippet exports retain their previous representation.
The new publication document is a separate action.
