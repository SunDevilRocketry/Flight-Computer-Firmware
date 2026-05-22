## Description
A brief description of the changes in the PR

### Issue Link
Please provide a link to the issue (e.g. "Closes #1"). 

Also, if this PR is one of multiple for this issue, link the parent if this is a child OR link 
the children if this is the parent. Do not use "closes" keywords on child PRs, only use them on 
the parent.

### Testing
- [ ] Passes existing unit tests
- [ ] Unit tests modified
- [ ] Integration test performed

If a non-automated test was executed that provides an artifact, please attach below (e.g. flash extract data).

### Other
Leave any additional notes here

## Reviewer Checklist

### Standards
- [ ] Follows FCF Architectural Standards
- [ ] Follows SDR Coding Standards
- [ ] Code complexity/function Size is minimized
- [ ] Code is testable
- [ ] Code is readable and commented properly
- [ ] License terms are respected

### Accuracy
- [ ] Code implements the correct requirement (a.k.a. does the right thing)
- [ ] Code is logically correct (a.k.a. does the thing right)

### Error Handling
- [ ] Potentially unsafe functions return a status code
- [ ] Error returns properly handled
- [ ] Fail-fast errors are only thrown when unsafe to continue software execution

### Memory
- [ ] Stack allocated memory is scoped correctly
- [ ] Heap allocated memory is not used
- [ ] Statically/Globally allocated memory is minimized except when necessary
- [ ] Pointers are used correctly
- [ ] Concurrent access has been considered

### Performance
- [ ] Rate limiters are respected
- [ ] Busy waiting is avoided in performance sensitive code
- [ ] "Delay" calls are not used in performance sensitive code
- [ ] If performance is negatively impacted, a justification is provided