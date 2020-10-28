%% closedLoopSystemLin
% constructs a linear system for the closed-loop
% dynamics
%
%% Syntax
%
% <html>
%        <div class="syntax">
% dx = closedLoopSystemLin(A,B,D,c,K,u_ref) <br>
%        </div>
% </html>
%
%% Description
% This function contructs a linear system object for the closed-loop
% dynamics for the optimization based controller from the system
% matrices of the uncontrolled system and the control law parameters.
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
%            A
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            system matrix of the open-loop system
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            B
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input matrix of the open-loop system
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            D
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            disturbance matrix of the open-loop system
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            c
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            constant offset vector of the open-loop system
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            K
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            feedback matrix for the control law u = u_ref + K(x-x_ref)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u_ref
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference input for the control law u = u_ref + K(x-x_ref)
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
%            sys
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            linear system object for the closed-loop dynamics
%            (class: linearSys)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <optimizationBasedControlhelp.html |optimizationBasedControl|>
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
