%% visualizeCommonRoad
% visualize a planned trajectory for different times
%
%% Syntax
%
% <html>
%        <div class="syntax">
% visualizeCommonRoad(obj,ind,dynObs,x0,lanelets,time) <br>
%        </div>
% </html>
%
%% Description
% This function visualizes the planned trajectory for a CommonRoad
% traffic scenario for the given time interval.
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
%            object of class maneuverAutomaton
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            ind
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector storing the indices of the motion-primitives for
%            the planned trajectory
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
%            cell-array storing the dynamic obstacles for the
%            CommonRoad traffic scenario
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
%            initial state for the CommonRoad traffic scenario
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            lanelets
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the lanelets for the CommonRoad
%            traffic scenario
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            time
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time interval for which the trajectory is plotted
%            (class: interval)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <maneuverAutomatonhelp.html |maneuverAutomaton|>
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
