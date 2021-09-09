/**
 * @name Resource not released in destructor or finalize()
 * @description All resources acquired by a class should be released by its
 * destructor. Avoid the use of the 'open / close' pattern, since C++
 * constructors and destructors provide a safer way to handle resource
 * acquisition and release. Best practice in C++ is to use the 'RAII'
 * technique: constructors allocate resources and destructors free them.
 * This version contains an exception for Fawkes, which allows resources
 * to be released in a finalize() method (of a Thread).
 * Cf. https://discuss.lgtm.com/t/multiple-c-false-positives/1451
 * and https://lgtm.com/query/projects:1506570706587/lang:cpp/
 * @kind problem
 * @problem.severity warning
 * @precision high
 * @id cpp/resource-not-released-in-destructor-or-finalize
 * @tags efficiency
 *       readability
 *       external/cwe/cwe-404
 */
import cpp

/**
 * Provides predicates for associating new/malloc calls with delete/free.
 *
 * This is a local copy of Critical/NewDelete.qll.
 */
module NewDelete_copy {
  import semmle.code.cpp.controlflow.SSA
  import semmle.code.cpp.dataflow.DataFlow

  /**
   * Holds if `alloc` is a use of `malloc` or `new`.  `kind` is
   * a string describing the type of the allocation.
   */
  predicate allocExpr(Expr alloc, string kind) {
    (
      exists(Function target |
        alloc.(AllocationExpr).(FunctionCall).getTarget() = target and
        (
          target.getName() = "operator new" and
          kind = "new" and
          // exclude placement new and custom overloads as they
          // may not conform to assumptions
          not target.getNumberOfParameters() > 1
          or
          target.getName() = "operator new[]" and
          kind = "new[]" and
          // exclude placement new and custom overloads as they
          // may not conform to assumptions
          not target.getNumberOfParameters() > 1
          or
          not target instanceof OperatorNewAllocationFunction and
          kind = "malloc"
        )
      )
      or
      alloc instanceof NewExpr and
      kind = "new" and
      // exclude placement new and custom overloads as they
      // may not conform to assumptions
      not alloc.(NewExpr).getAllocatorCall().getTarget().getNumberOfParameters() > 1
      or
      alloc instanceof NewArrayExpr and
      kind = "new[]" and
      // exclude placement new and custom overloads as they
      // may not conform to assumptions
      not alloc.(NewArrayExpr).getAllocatorCall().getTarget().getNumberOfParameters() > 1
    ) and
    not alloc.isFromUninstantiatedTemplate(_)
  }

  /**
   * Holds if `alloc` is a use of `malloc` or `new`, or a function
   * wrapping one of those.  `kind` is a string describing the type
   * of the allocation.
   */
  predicate allocExprOrIndirect(Expr alloc, string kind) {
    // direct alloc
    allocExpr(alloc, kind)
    or
    exists(ReturnStmt rtn |
      // indirect alloc via function call
      alloc.(FunctionCall).getTarget() = rtn.getEnclosingFunction() and
      (
        allocExprOrIndirect(rtn.getExpr(), kind)
        or
        exists(Expr e |
          allocExprOrIndirect(e, kind) and
          DataFlow::localExprFlow(e, rtn.getExpr())
        )
      )
    )
  }

  /**
   * Holds if `v` is a non-local variable which is assigned with allocations of
   * type `kind`.
   */
  pragma[nomagic]
  private predicate allocReachesVariable(Variable v, Expr alloc, string kind) {
    exists(Expr mid |
      not v instanceof StackVariable and
      v.getAnAssignedValue() = mid and
      allocReaches0(mid, alloc, kind)
    )
  }

  /**
   * Holds if `e` is an expression which may evaluate to the
   * result of a previous memory allocation `alloc`.  `kind` is a
   * string describing the type of that allocation.
   */
  private predicate allocReaches0(Expr e, Expr alloc, string kind) {
    // alloc
    allocExprOrIndirect(alloc, kind) and
    e = alloc
    or
    exists(SsaDefinition def, StackVariable v |
      // alloc via SSA
      allocReaches0(def.getAnUltimateDefiningValue(v), alloc, kind) and
      e = def.getAUse(v)
    )
    or
    exists(Variable v |
      // alloc via a global
      allocReachesVariable(v, alloc, kind) and
      strictcount(VariableAccess va | va.getTarget() = v) <= 50 and // avoid very expensive cases
      e.(VariableAccess).getTarget() = v
    )
  }

  /**
   * Holds if `free` is a use of free or delete.  `freed` is the
   * expression that is freed / deleted and `kind` is a string
   * describing the type of that free or delete.
   */
  predicate freeExpr(Expr free, Expr freed, string kind) {
    exists(Function target |
      freed = free.(DeallocationExpr).getFreedExpr() and
      free.(FunctionCall).getTarget() = target and
      (
        target.getName() = "operator delete" and
        kind = "delete"
        or
        target.getName() = "operator delete[]" and
        kind = "delete[]"
        or
        not target instanceof OperatorDeleteDeallocationFunction and
        kind = "free"
      )
    )
    or
    free.(DeleteExpr).getExpr() = freed and
    kind = "delete"
    or
    free.(DeleteArrayExpr).getExpr() = freed and
    kind = "delete[]"
  }

  /**
   * Holds if `free` is a use of free or delete, or a function
   * wrapping one of those.  `freed` is the expression that is
   * freed / deleted and `kind` is a string describing the type
   * of that free or delete.
   */
  predicate freeExprOrIndirect(Expr free, Expr freed, string kind) {
    // direct free
    freeExpr(free, freed, kind)
    or
    // indirect free via function call
    exists(Expr internalFree, Expr internalFreed, int arg |
      freeExprOrIndirect(internalFree, internalFreed, kind) and
      free.(FunctionCall).getTarget().getParameter(arg) = internalFreed.(VariableAccess).getTarget() and
      free.(FunctionCall).getArgument(arg) = freed
    )
  }
}

import NewDelete_copy

/**
 * An expression that acquires a resource, and the kind of resource that is acquired.  The
 * kind of a resource indicates which acquisition/release expressions can be paired.
 */
predicate acquireExpr(Expr acquire, string kind) {
  exists(FunctionCall fc, Function f, string name |
    fc = acquire and
    f = fc.getTarget() and
    name = f.getName() and 
    (
      (
        name = "fopen" and
        kind = "file"
      ) or (
        name = "open" and
        kind = "file descriptor"
      ) or (
        name = "socket" and
        kind = "file descriptor"
      )
    )
  ) or (
    allocExpr(acquire, kind)
  )
}

/**
 * An expression that releases a resource, and the kind of resource that is released.  The
 * kind of a resource indicates which acquisition/release expressions can be paired.
 */
predicate releaseExpr(Expr release, Expr resource, string kind) {
  exists(FunctionCall fc, Function f, string name |
    fc = release and
    f = fc.getTarget() and
    name = f.getName() and 
    (
      (
        name = "fclose" and
        resource = fc.getArgument(0) and
        kind = "file"
      ) or (
        name = "close" and
        resource = fc.getArgument(0) and
        kind = "file descriptor"
      )
    )
  ) or exists(string releaseKind |
    freeExpr(release, resource, releaseKind) and
    (
      (
        kind = "malloc" and
        releaseKind = "free"
      ) or (
        kind = "new" and
        releaseKind = "delete"
      ) or (
        kind = "new[]" and
        releaseKind = "delete[]"
      )
    )
  )
}

/**
 * Gets the expression `e` or a `PointerDereferenceExpr` around
 * it.
 */
Expr exprOrDereference(Expr e) {
  result = e or
  result.(PointerDereferenceExpr).getOperand() = e
}

/**
 * Holds if the expression `e` releases expression `released`, whether directly
 * or via one or more function call(s).
 */
private predicate exprReleases(Expr e, Expr released, string kind) {
  (
    // `e` is a call to a release function and `released` is the released argument
    releaseExpr(e, released, kind)
  ) or exists(Function f, int arg |
    // `e` is a call to a function that releases one of it's parameters,
    // and `released` is the corresponding argument
    e.(FunctionCall).getTarget() = f and
    e.(FunctionCall).getArgument(arg) = released and
    exprReleases(_, exprOrDereference(f.getParameter(arg).getAnAccess()), kind)
  ) or exists(Function f, ThisExpr innerThis |
    // `e` is a call to a method that releases `this`, and `released`
    // is the object that is called
    e.(FunctionCall).getTarget() = f and
    e.(FunctionCall).getQualifier() = exprOrDereference(released) and
    innerThis.getEnclosingFunction() = f and
    exprReleases(_, innerThis, kind)
  )
}

class FawkesDestructor extends MemberFunction {
  FawkesDestructor() {
    this instanceof Destructor
    or
    this.getName() = "finalize"
  }
}

class Resource extends MemberVariable {

  Resource() { not isStatic() }

  // Check that an expr is somewhere in this class - does not have to be a constructor
  predicate inSameClass(Expr e) {
    e.getEnclosingFunction().(MemberFunction).getDeclaringType() = this.getDeclaringType()
  }

  private predicate calledFromDestructor(Function f) {
    (f instanceof FawkesDestructor and f.getDeclaringType() = this.getDeclaringType())
    or
    exists(Function mid, FunctionCall fc |
      calledFromDestructor(mid) and
      fc.getEnclosingFunction() = mid and
      fc.getTarget() = f and
      f.getDeclaringType() = this.getDeclaringType())
  }

  predicate inDestructor(Expr e) {
    exists(Function f | f = e.getEnclosingFunction() |
      calledFromDestructor(f)
    )
  }

  predicate acquisitionWithRequiredRelease(Assignment acquireAssign, string kind) {
    // acquireAssign is an assignment to this resource
    acquireAssign.(Assignment).getLValue() = this.getAnAccess() and
    // Should be in this class, but *any* member method will do
    this.inSameClass(acquireAssign) and
    // Check that it is an acquisition function and return the corresponding free
    acquireExpr(acquireAssign.getRValue(), kind)
  }

  Expr getAReleaseExpr(string kind) {
    exprReleases(result, this.getAnAccess(), kind)
  }
}

predicate unreleasedResource(Resource r, Expr acquire, File f, int acquireLine) {
    // Note: there could be several release functions, because there could be
    // several functions called 'fclose' for example. We want to check that
    // *none* of these functions are called to release the resource
    r.acquisitionWithRequiredRelease(acquire, _) and
    not exists(Expr releaseExpr, string releaseName |
         r.acquisitionWithRequiredRelease(acquire, releaseName) and
         releaseExpr = r.getAReleaseExpr(releaseName) and
         r.inDestructor(releaseExpr)
    )
    and f = acquire.getFile()
    and acquireLine = acquire.getLocation().getStartLine()

    // check that any destructor for this class has a block; if it doesn't,
    // we must be missing information.
    and forall(Class c, FawkesDestructor d |
      r.getDeclaringType().isConstructedFrom*(c) and
      d = c.getAMember() and
      not d.isCompilerGenerated() and
      not d.isDefaulted() and
      not d.isDeleted() |
      exists(d.getBlock())
    ) 
}

predicate freedInSameMethod(Resource r, Expr acquire) {
  unreleasedResource(r, acquire, _, _) and
  exists(Expr releaseExpr, string releaseName |
    r.acquisitionWithRequiredRelease(acquire, releaseName) and
    releaseExpr = r.getAReleaseExpr(releaseName) and
    releaseExpr.getEnclosingFunction() = acquire.getEnclosingFunction()
  )
}

/**
 * Resource `r`, acquired by `acquire`, is passed to some external
 * object in the function where it's acquired.  This other object
 * may have taken responsibility for freeing the resource.
 */
predicate leakedInSameMethod(Resource r, Expr acquire) {
  unreleasedResource(r, acquire, _, _) and
  (
    exists(FunctionCall fc |
      // `r` (or something computed from it) is passed to another function
      // near to where it's acquired, and might be stored elsewhere.
      fc.getAnArgument().getAChild*() = r.getAnAccess() and
      fc.getEnclosingFunction() = acquire.getEnclosingFunction()
    ) or exists(Variable v, Expr e | 
      // `r` (or something computed from it) is stored in another variable
      // near to where it's acquired, and might be released through that
      // variable.
      v.getAnAssignedValue() = e and
      e.getAChild*() = r.getAnAccess() and
      e.getEnclosingFunction() = acquire.getEnclosingFunction()
    ) or exists(FunctionCall fc |
      // `this` (i.e. the class where `r` is acquired) is passed into `r` via a
      // method, or the constructor.  `r` may use this to register itself with
      // `this` in some way, ensuring it is later deleted.
      fc.getEnclosingFunction() = acquire.getEnclosingFunction() and
      fc.getAnArgument() instanceof ThisExpr and
      (
        fc.getQualifier() = r.getAnAccess() or // e.g. `r->setOwner(this)`
        fc = acquire.getAChild*() // e.g. `r = new MyClass(this)`
      )
    )
  )
}

pragma[noopt] predicate badRelease(Resource r, Expr acquire, Function functionCallingRelease, int line) {
  unreleasedResource(r, acquire, _, _) and
  exists(Expr releaseExpr, string releaseName,
         Location releaseExprLocation, Function acquireFunction |
    r.acquisitionWithRequiredRelease(acquire, releaseName) and
    releaseExpr = r.getAReleaseExpr(releaseName) and
    releaseExpr.getEnclosingFunction() = functionCallingRelease and
    functionCallingRelease.getDeclaringType() = r.getDeclaringType() and
    releaseExprLocation = releaseExpr.getLocation() and
    line = releaseExprLocation.getStartLine() and
    acquireFunction = acquire.getEnclosingFunction() and
    functionCallingRelease != acquireFunction
  )
}

Class qtObject() { result.getABaseClass*().getQualifiedName() = "QObject" }
PointerType qtObjectReference() { result.getBaseType() = qtObject() }
Constructor qtParentConstructor() {
  exists(Parameter p |
    p.getName() = "parent" and
    p.getType() = qtObjectReference() and
    result.getAParameter() = p and
    result.getDeclaringType() = qtObject()
  )
}

predicate automaticallyReleased(Assignment acquire)
{
  // sub-types of the Qt type QObject are released by their parent (if they have one)
  exists(NewExpr alloc |
    alloc.getType() = qtObject() and
    acquire.getRValue() = alloc and
    alloc.getInitializer() = qtParentConstructor().getACallToThisFunction()
  )
}

from Resource r, Expr acquire, File f, string message
where unreleasedResource(r, acquire, f, _) and
      not freedInSameMethod(r, acquire) and
      not leakedInSameMethod(r, acquire) and
      (
        exists(Function releaseFunction, int releaseLine | badRelease(r, acquire, releaseFunction, releaseLine) and
          message =
          "Resource " + r.getName() + " is acquired by class " + r.getDeclaringType().getName() +
          " but not released in the destructor. It is released from " + releaseFunction.getName() + " on line " + releaseLine +
          ", so this function may need to be called from the destructor or finalize."
        )
        or
        (
          not badRelease(r, _, _, _) and
          message = "Resource " + r.getName() + " is acquired by class " + r.getDeclaringType().getName() + " but not released anywhere in this class."
        )
      ) and
      not automaticallyReleased(acquire) and
      not r.getDeclaringType() instanceof TemplateClass // template classes may contain insufficient information for this analysis; results from instantiations will usually suffice.
select acquire, message

