%% reachSet
% computes the reachable set for an MPC comfort controller
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [res,R,Param] = reachSet(obj,x0,iter) <br>
%        </div>
% </html>
%
%% Description
% Computes the reachable set for one time step of the MPC
% (Model Predictive Control) controller which is used as a comfort
% controller during online application of the Safety Net Controller.
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
%            object of class comfContrMPC storing the control law
%            computed in the offline-phase
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
%            initial point for the reachable set computation
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            iter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            current iteration (=time step) of the Safety Net Controller
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
%            res
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            flag specifying if the constraints are satisfied or not
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final reachable set (class: zonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Param
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            control parameters computed during the online phase which
%            are required for the simulation of the controller later on
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <safetyNetControlhelp.html |safetyNetControl|>, <simulatehelp.html |simulate|>, <comfContrMPChelp.html |comfContrMPC|>
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
