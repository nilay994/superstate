function getCoord(aH,evnt, arg1, arg2)
drawnow
f = ancestor(aH,'figure');
click_type = get(f,'SelectionType');
ptH = getappdata(aH,'CurrentPoint');
% delete(ptH)
if strcmp(click_type,'normal')
    %Finding the closest point and highlighting it
    lH = findobj(aH,'Type','line');
    minDist = realmax;
    finalIdx = NaN;
    finalH = NaN;
    pt = get(aH,'CurrentPoint'); %Getting click position
    for ii = lH'
        xp=get(ii,'Xdata'); %Getting coordinates of line object
        yp=get(ii,'Ydata');
        dx=daspect(aH);      %Aspect ratio is needed to compensate for uneven axis when calculating the distance
        [newDist idx] = min(((pt(1,1)-xp).*dx(2)).^2 + ((pt(1,2)-yp).*dx(1)).^2 );
        if (newDist < minDist)
            finalH = ii;
            finalIdx = idx;
            minDist = newDist;
        end
    end
    xp=get(finalH,'Xdata'); %Getting coordinates of line object
    yp=get(finalH,'Ydata');
    % ptH = plot(aH,xp(finalIdx),yp(finalIdx),'k*','MarkerSize',20);
    x1 = xp(finalIdx) - 0.15;
    x2 = xp(finalIdx) + 0.15;    
    y1 = yp(finalIdx);
    y2 = yp(finalIdx);
    
    x3 = xp(finalIdx);
    x4 = xp(finalIdx);
    y3 = yp(finalIdx);
    y4 = yp(finalIdx) + arg2(finalIdx);
    
    phi = -1 * arg1(finalIdx, 1);
    R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    
    newmat1 = R * [x1-xp(finalIdx); y1-yp(finalIdx)];
    newmat2 = R * [x2-xp(finalIdx); y2-yp(finalIdx)];
    
    newmat3 = R * [x3-xp(finalIdx); y3-yp(finalIdx)];
    newmat4 = R * [x4-xp(finalIdx); y4-yp(finalIdx)];
    
    newx1 = newmat1(1);
    newy1 = newmat1(2);
    
    newx2 = newmat2(1);
    newy2 = newmat2(2);
    
    newx3 = newmat3(1);
    newy3 = newmat3(2);
    
    newx4 = newmat4(1);
    newy4 = newmat4(2);
    
    ptH = plot(aH, [newx1+xp(finalIdx) newx2+xp(finalIdx)], [newy1+yp(finalIdx) newy2+yp(finalIdx)], '-r'); hold on;
    ptH = plot(aH, [newx3+xp(finalIdx) newx4+xp(finalIdx)], [newy3+yp(finalIdx) newy4+yp(finalIdx)], '-r');
    text(newx4+xp(finalIdx), newy4+yp(finalIdx), num2str(arg2(finalIdx)), 'BackgroundColor', 'w');
    setappdata(aH,'CurrentPoint',ptH);
elseif strcmp(click_type,'alt')
    %do your stuff once your point is selected   
    disp('Done clicking!');
    % HERE IS WHERE YOU CAN PUT YOUR STUFF
    uiresume(f);
end