%% plotSimulation
% plot all simulated trajectories
%
%% Syntax
%
% <html>
%        <div class="syntax">
% han = plotSimulation(obj) <br>
% han = plotSimulation(obj,dim) <br>
% han = plotSimulation(obj,dim,color) <br>
% han = plotSimulation(obj,dim,color,options) <br>
%        </div>
% </html>
%
%% Description
% Plot all simulated trajectories that are stored in the results
% object.
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
%            object of class results
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            dim
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            2D-vector containing the dimensions that are plotted
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            color
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            color for the plot specified as a string (e.g. <tt>'r'</tt>) or
%            as a vector of RGB-values (e.g. [0 1 0])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            options
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            additional MATLAB plot options specified as name-value
%            pairs (e.g. <tt>'LineWidth'</tt>,2)
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
%            han
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            handle pointing to the plotted trajectories
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <resultshelp.html |results|>, <plotReachhelp.html |plotReach|>, <simulatehelp.html |simulate|>, <simulateRandomhelp.html |simulateRandom|>
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
