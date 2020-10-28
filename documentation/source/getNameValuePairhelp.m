%% getNameValuePair
% returns the value of a name-value-pair for plotting
%
%% Syntax
%
% <html>
%        <div class="syntax">
% val = getNameValuePair(NVpairs,name) <br>
% val = getNameValuePair(NVpairs,name,defVal) <br>
%        </div>
% </html>
%
%% Description
% This function takes a list of name-value-pairs specified by the
% user as additional options for a plot function (e.g. |'LineWidth'|,2)
% and returns the value of the selected option.
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
%            NVpairs
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the name-value-pairs passed to the
%            plot function
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            name
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            string containing the name for the name-value-pair
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            defVal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            default value if the name value pair is not specified
%            by the user
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
%            val
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            specified value for the name value pair
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <resultshelp.html |results|>, <plotReachhelp.html |plotReach|>, <plotReachTimePointhelp.html |plotReachTimePoint|>, <plotSimulationhelp.html |plotSimulation|>
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
