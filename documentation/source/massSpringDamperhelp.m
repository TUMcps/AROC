%% massSpringDamper
% dynamic equations for the mass-spring-damper benchmark
% model
%
%% Syntax
%
% <html>
%        <div class="syntax">
% f = massSpringDamper(x, u, w) <br>
%        </div>
% </html>
%
%% Description
% Dynamic equation for the mass-spring-damper benchmark model. The
% benchmark represents a rigid body that is coupled to the
% environment by a spring and a damper, where both the stiffness of
% the spring and the damping of the damper are nonlinear functions of
% the rigid body position. The system states are the position and the
% velocity of the rigid body. The input to the system is an external
% force acting on the rigid body
%
% <<img/massSpringDamper.png>>
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
%            x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            system states x (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            control inputs u (dimension: [nu,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            w
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            disturbances w (dimension: [nw,1])
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
%            f
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            value of the dynamic function = time derivative of x
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <carhelp.html |car|>, <doubleIntegratorhelp.html |doubleIntegrator|>, <robothelp.html |robot|>
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
