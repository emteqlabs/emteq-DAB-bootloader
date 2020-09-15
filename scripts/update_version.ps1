#requires -version 2
<#
.SYNOPSIS
  <Overview of script>

.DESCRIPTION
  <Brief description of script>

.PARAMETER <Parameter_Name>
    <Brief description of parameter input required. Repeat this attribute if required>
    
.PARAMETER -rc
   For Resource (.rc) file define as preprocessor variable

.INPUTS
  <Inputs if any, otherwise state None>

.OUTPUTS
  <Outputs if any, otherwise state None - example: Log file stored in C:\Windows\Temp\<name>.log>

.NOTES
  Version:        1.0
  Author:         <Name>
  Creation Date:  <Date>
  Purpose/Change: Initial script development
  
.EXAMPLE
  <Example goes here. Repeat this attribute for more than one example>
#>
param (
    [String]$filename = "Version.generated.h",
    [switch]$rc = $false
)

Begin 
{
    Write-Host "Generating version file '$filename'."
}
Process 
{    
    try
    {
        # Ensure the repo hooks are setup so this script is called after every commit
        git config core.hooksPath .githooks

        $buildTag = git --no-pager describe --tags --always --dirty --long
        $semantictag,$commits,$sha,$dirty = $buildTag.trim('v').split('-')
        $major,$minor,$patch,$others = [int[]]$semantictag.split('.') + [int]0; # @note Add an extra '0' at the end as the Patch build is optional i.e. 1.07 vs 1.07.5

        $semanticVersion = [version]("$major.$minor.$patch.$commits")

        $nl = [Environment]::NewLine
        if ( $rc )
        {
            $content = "#define VERSION_RC $major,$minor,$patch,$commits$nl" +`
                       '#define VERSION_STR "' + $buildTag + '"'
        }
        else
        {
            $content =  "#define PROJECT_VERSION_MAJOR $major$nl" +`
                        "#define PROJECT_VERSION_MINOR $minor$nl" +`
                        "#define PROJECT_VERSION_PATCH $patch$nl" +`
                        "#define PROJECT_VERSION_COMMIT $commits$nl" +`
                        "#define PROJECT_VERSION_SHA $sha$nl" +`
                        '#define PROJECT_VERSION "' + $semanticVersion + '"' + $nl +`
                        '#define PROJECT_FULLVERSION "' + $buildTag + '"' + $nl
        }

        $content | sc $filename
    }
    catch
    {
        Write-Error $_.Exception.ToString()
        pause
    }
}
End
{
    $filepath = Resolve-Path $filename
    Write-Host "Version set to '$buildTag' [$semanticVersion] in '$filepath'."
    Write-Host "Please recompile the binary to reflect the new version-tag."  -Foreground green
    Exit 0
}