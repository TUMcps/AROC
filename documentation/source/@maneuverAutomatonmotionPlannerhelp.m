%% motionPlanner
% plan a verified trajectoy with a Maneuver Automaton
%
%% Syntax
%
% <html>
%        <div class="syntax">
% ind = motionPlanner(obj,x0,goalSet,statObs,dynObs,search) <br>
%        </div>
% </html>
%
%% Description
% This function solves a control task by planning a verified
% trajectoy with a Maneuver Automaton. In particular, the verified
% trajectory avoids the static and dynamic obstacles and drives the
% system to the goal set.
%
%% Input Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            obj
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Maneuver automaton (class: maneuverAutomaton)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial state
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            goalSet
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            goal set which should be reached
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            statObs
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the static obstacles
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            dynObs
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the dynamic obstacles
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            search
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            search algorithm to be used (<tt>'breadth-first'</tt>, ...;
%            <tt>'depth-first'</tt>, or <tt>'Astar'</tt>)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% Output Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            ind
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector storing the indices of the motion primitives
%            that are concatanated to obtain the verified trajectory
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <matlab:doc('maneuverAuotomaton') maneuverAuotomaton>
%

%%
% <html>
%   <hr>
%   <p class="copy">&copy; 2018-2020 I6 Technische Universit&auml;t M&uuml;nchen
%        <tt class="minicdot">&#149;</tt>
%        <a href="https://tumcps.github.io/AROC/">Website</a>
%        <tt class="minicdot">&#149;</tt>
%        <a href="file:txts/LICENSE.txt">License</a>
%   </p>
% <div>
% <table>
%  <tr>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%             <img src="img/logoAroc.png" alt="logoAroc" height="40px">
%      </td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoCora.png" alt="logoCora" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoChair.png" alt="logoChair" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoTum.png" alt="logoTum" height="40px"></td>
%  </tr>
% </table>
% </div>
% </html>
