%% results
% class storing the reachalbe set and/or simulation of the
% computted control law
%
%% Syntax
%
% <html>
%        <div class="syntax">
% obj = results(reachSet,reachSetTimePoint,refTraj) <br>
% obj = results(reachSet,reachSetTimePoint,refTraj,simulation) <br>
%        </div>
% </html>
%
%% Description
% Constructor of the results objects that stores the computed
% reachable set, the reference trajectory and/or simulated
% trajectories for the control law.
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
%            reachSet
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachable set for the computed maneuver
%            (class: reachSet)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            reachSetTimePoint
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachable set for the computed maneuver at
%            the begging of each center trajectory time
%            step
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            refTraj
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference trajectory for the maneuver
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            simulation
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            data from the simulation of the system
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% Output Arguments
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
%            resulting object of class results
%        </p>

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
