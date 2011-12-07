<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">
    <!-- general formatring options -->
    <xsl:output indent="yes"/>
    <xsl:strip-space  elements="*"/>

    <!-- Remove HEAD constants -->
    <xsl:template match="/plan/head/item/b"/>
    <xsl:template match="/plan/head/item/bcx"/>
    <xsl:template match="/plan/head/item/bcy"/>
    <xsl:template match="/plan/head/item/dmin"/>
    <xsl:template match="/plan/head/item/dmax"/>
    <xsl:template match="/plan/head/item/dcen"/>
    <xsl:template match="/plan/head/item/dh"/>
    <xsl:template match="/plan/head/item/eps"/>
    <!-- Remove PKM constants -->
    <xsl:template match="/plan/pkm/item/pkmbaseToFixedPkm"/>
    <xsl:template match="/plan/pkm/item/hlowered"/>
    <xsl:template match="/plan/pkm/item/Dz"/>
    <xsl:template match="/plan/pkm/item/maxPkmNeutralDist"/>
    <xsl:template match="/plan/pkm/item/PX"/>
    <xsl:template match="/plan/pkm/item/PZ"/>
    <xsl:template match="/plan/pkm/item/COSGA"/>
    <xsl:template match="/plan/pkm/item/SINGA"/>
    <xsl:template match="/plan/pkm/item/baseToPkmAlfa"/>
    <xsl:template match="/plan/pkm/item/baseToPkmDx"/>
    <xsl:template match="/plan/pkm/item/baseToPkmDz"/>
    <!-- Remove PKM joint coordinates -->
    <xsl:template match="/plan/pkm/item/alpha0"/>    
    <xsl:template match="/plan/pkm/item/l1"/>       
    <xsl:template match="/plan/pkm/item/l2"/>
    <xsl:template match="/plan/pkm/item/l3"/>
    <xsl:template match="/plan/pkm/item/psi1"/>
    <xsl:template match="/plan/pkm/item/psi2"/>
    <xsl:template match="/plan/pkm/item/psi3"/>
    <xsl:template match="/plan/pkm/item/beta7"/>
    
    
    <!-- Rename ox,oy,oz to alpha,beta,gamma -->
    <xsl:template match="/plan/pkm/item/Xyz_Euler_Zyz/ox">
        <alpha>
            <xsl:apply-templates select="node()"/>
        </alpha>
    </xsl:template>
    <xsl:template match="/plan/pkm/item/Xyz_Euler_Zyz/oy">
        <beta>
            <xsl:apply-templates select="node()"/>
        </beta>
    </xsl:template>
    <xsl:template match="/plan/pkm/item/Xyz_Euler_Zyz/oz">
        <gamma>
            <xsl:apply-templates select="node()"/>
        </gamma>
    </xsl:template>
    
    <!-- Choose between Homog_matrix and XYZ_Euler_Zyz representation -->
    <!--
    <xsl:template match="/plan/pkm/item/Xyz_Euler_Zyz"/>
    -->
    <xsl:template match="/plan/pkm/item/pkmToWrist"/>
    
    <!-- default action is to copy all elements --> 
    <xsl:template match="node()">
        <xsl:copy>
            <xsl:apply-templates select="node()"/>
        </xsl:copy>
    </xsl:template>

    <!-- reorder fields of base 'State' class -->
    <xsl:template match="/plan/pkm/item|/plan/head/item|/plan/mbase/item">
        <item>
            <xsl:apply-templates select="agent"/>
            <xsl:apply-templates select="TBeg"/>
            <xsl:apply-templates select="TEnd"/>
            <xsl:apply-templates select="node()[not(name()='agent')][not(name()='TEnd')][not(name()='TBeg')]"/>
        </item>
    </xsl:template>
    
    <!-- Remove lists lengths --> 
    <xsl:template match="/plan/sNum"/>
    <xsl:template match="/plan/hNum"/>
    <xsl:template match="/plan/bNum"/>
    <xsl:template match="/plan/pNum"/>
    
    <!-- Remove empty list items -->
    <xsl:template match="/plan/mbase/item/actions">
        <xsl:variable name="nA"><xsl:value-of select="../numActions"/></xsl:variable>
        <actions>
            <xsl:apply-templates select="item[position() &lt; $nA+1]"/>
        </actions>
    </xsl:template>
    
    <!-- Remove unneeded data -->
    <xsl:template match="/plan/svar"/>
    <xsl:template match="/plan/trajectory"/>
    <xsl:template match="/plan/workpiece"/>
    <xsl:template match="/plan/MaxHeadPerSegNum"/>
    <xsl:template match="/plan/MaxPkmPerHeadNum"/>      

</xsl:stylesheet>