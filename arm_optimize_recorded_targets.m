function result = arm_optimize_recorded_targets(p,targets,options)
% Expand the search if the initial grid cannot satisfy all recorded targets.
if nargin<3, options=struct(); end
options=put(options,'L1Values',.20:.01:.60);
options=put(options,'L2Values',.20:.01:.60);
options=put(options,'autoExpand',true);
options=put(options,'maxLinkLength',max([1 options.L1Values(:)' options.L2Values(:)']));
options=put(options,'expansionIncrement',.20);
validateattributes(options.maxLinkLength,{'numeric'},{'real','finite','positive','scalar'});
validateattributes(options.expansionIncrement,{'numeric'},{'real','finite','positive','scalar'});
initial=options;
if options.autoExpand
 assert(options.maxLinkLength>=max([options.L1Values(:);options.L2Values(:)]), ...
  'Auto-expand ceiling must be at least the initial maximum length.');
end
attempts=0;
while true
 attempts=attempts+1;
 result=arm_optimize_lengths(p,targets,options);
 if ~isempty(result.bestP) || ~options.autoExpand, break; end
 old1=max(options.L1Values); old2=max(options.L2Values);
 cap=options.maxLinkLength;
 if old1>=cap-1e-10 && old2>=cap-1e-10, break; end
 options.L1Values=extend(initial.L1Values,min(cap,old1+options.expansionIncrement));
 options.L2Values=extend(initial.L2Values,min(cap,old2+options.expansionIncrement));
end
result.expansion=struct('attempts',attempts,'initialMax',[max(initial.L1Values) max(initial.L2Values)], ...
 'searchedMax',[max(options.L1Values) max(options.L2Values)],'ceiling',options.maxLinkLength);
if attempts>1
 result.message=sprintf('%s Search expanded to L1 %.3f / L2 %.3f m.',result.message, ...
  result.expansion.searchedMax);
end
end
function s=put(s,k,value)
if ~isfield(s,k), s.(k)=value; end
end
function values=extend(original,upper)
original=sort(unique(original(:)')); step=.01;
if numel(original)>1, step=min(diff(original)); end
% Merge grid points within 1 pm; roundoff must not create duplicate candidates.
values=unique(round([original min(original):step:upper upper]*1e12)/1e12);
end
