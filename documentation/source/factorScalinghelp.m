%% factorScaling
% compute optimal scaling factor for all generators
%
%% Syntax
%
% <html>
%        <div class="syntax">
% s = factorScaling(R0,Rfin,H,C,d) <br>
%        </div>
% </html>
%
%% Description
% This function computes the optimal scaling factors for all
% generators such that the volume of the initial set is maximized
% and the final reachable set is fully contained in the goal set. The
% optimal scaling factors are determined by nonlinear programming
% using the "fmincon" algorithm.
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
%            R0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial set (class: zonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Rfin
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final forward reachable set (class: polyZonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            H
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            matrix storing the generator-to-input assignment
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            C
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            matrix for the inequality constraint C*x <= d
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            d
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector for the inequality constraint C*x <= d
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
%            s
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector with the optimal scaling factor for each generator
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <safetyNetControlhelp.html |safetyNetControl|>
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
