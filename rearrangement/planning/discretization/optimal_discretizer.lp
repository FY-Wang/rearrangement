% grid optimization

% input describes a nonoptimal grid of size maxx * maxy with objects
%    input is given in a file, say input_grid.lp
% output describes an optimal grid (i.e., which horizontal/vertical
%    lines of the input grid to keep)
%    output can be written in a file, say output_grid.lp

% command line: clingo input_grid.lp grid_optimizer.lp > output_grid.lp

% generate an optimal grid

% first, include the border lines of the input grid
newvline(0). newvline(maxx).
newhline(0). newhline(maxy).

% next, pick some of the other horizontal/vertical grid lines of the input grid
{newhline(J) : hline(J), J>0, J<maxy}.
{newvline(J) : vline(J), J>0, J<maxx}.

% the grid cells of the new grid are defined by means of diagonally-opposite corners
cell(X1,Y1,X2,Y2) :-
   newvline(X1), newvline(X2), newhline(Y1), newhline(Y2),
   X1<X2, Y1<Y2, vline(X1), vline(X2), hline(Y1), hline(Y2),
   % no grid cell occupies/overlaps with another grid cell
   {newhline(Y) : hline(Y), Y1<Y, Y<Y2}0,
   {newvline(X) : vline(X), X1<X, X<X2}0.

% make sure that no cell of the new grid contains two or more objects
:- cell(X1,Y1,X2,Y2),
   2{obj(X3,Y3,X4,Y4) : X1<=X3, X2>=X4, Y1<=Y3, Y2>=Y4}.

% make sure that at least one cell of the new grid is empty
emptycell :-
   cell(X1,Y1,X2,Y2),
   {obj(X3,Y3,X4,Y4) : X1<=X3, X2>=X4, Y1<=Y3, Y2>=Y4}0.
:- not emptycell.

% make sure that the number of the inner horizontal/vertical grid lines in the new grid is minimum
:~ newhline(J). [1@1, J]
:~ newvline(J). [1@1, J]

#show newhline/1.
#show newvline/1.