<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis.dtd' 'SYSTEM'>
<qgis readOnly="0" maxScale="0" hasScaleBasedVisibilityFlag="0" simplifyDrawingHints="0" labelsEnabled="0" simplifyAlgorithm="0" simplifyMaxScale="1" version="3.3.0-Master" simplifyLocal="1" simplifyDrawingTol="1" minScale="1e+08">
  <renderer-v2 symbollevels="0" forceraster="0" enableorderby="0" type="singleSymbol">
    <symbols>
      <symbol name="0" alpha="1" clip_to_extent="1" type="marker">
        <layer pass="0" locked="0" enabled="1" class="SimpleMarker">
          <prop k="angle" v="0"/>
          <prop k="color" v="253,191,111,255"/>
          <prop k="horizontal_anchor_point" v="1"/>
          <prop k="joinstyle" v="bevel"/>
          <prop k="name" v="arrow"/>
          <prop k="offset" v="0,0"/>
          <prop k="offset_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="offset_unit" v="MM"/>
          <prop k="outline_color" v="0,0,0,255"/>
          <prop k="outline_style" v="solid"/>
          <prop k="outline_width" v="0"/>
          <prop k="outline_width_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="outline_width_unit" v="MM"/>
          <prop k="scale_method" v="area"/>
          <prop k="size" v="5"/>
          <prop k="size_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="size_unit" v="MM"/>
          <prop k="vertical_anchor_point" v="1"/>
          <data_defined_properties>
            <Option type="Map">
              <Option name="name" value="" type="QString"/>
              <Option name="properties" type="Map">
                <Option name="angle" type="Map">
                  <Option name="active" value="true" type="bool"/>
                  <Option name="expression" value="-(&quot;velocity_theta&quot; * 180 / 3.14159265) - 90" type="QString"/>
                  <Option name="type" value="3" type="int"/>
                </Option>
              </Option>
              <Option name="type" value="collection" type="QString"/>
            </Option>
          </data_defined_properties>
        </layer>
      </symbol>
    </symbols>
    <rotation/>
    <sizescale/>
  </renderer-v2>
  <customproperties>
    <property value="0" key="embeddedWidgets/count"/>
    <property key="variableNames"/>
    <property key="variableValues"/>
  </customproperties>
  <blendMode>0</blendMode>
  <featureBlendMode>0</featureBlendMode>
  <layerOpacity>1</layerOpacity>
  <SingleCategoryDiagramRenderer diagramType="Histogram" attributeLegend="1">
    <DiagramCategory labelPlacementMethod="XHeight" sizeType="MM" rotationOffset="270" minScaleDenominator="0" lineSizeScale="3x:0,0,0,0,0,0" penWidth="0" backgroundColor="#ffffff" scaleDependency="Area" penAlpha="255" enabled="0" minimumSize="0" lineSizeType="MM" opacity="1" penColor="#000000" maxScaleDenominator="1e+08" backgroundAlpha="255" scaleBasedVisibility="0" sizeScale="3x:0,0,0,0,0,0" diagramOrientation="Up" width="15" barWidth="5" height="15">
      <fontProperties description="Ubuntu,11,-1,5,50,0,0,0,0,0" style=""/>
    </DiagramCategory>
  </SingleCategoryDiagramRenderer>
  <DiagramLayerSettings showAll="1" linePlacementFlags="18" dist="0" obstacle="0" placement="0" priority="0" zIndex="0">
    <properties>
      <Option type="Map">
        <Option name="name" value="" type="QString"/>
        <Option name="properties"/>
        <Option name="type" value="collection" type="QString"/>
      </Option>
    </properties>
  </DiagramLayerSettings>
  <fieldConfiguration>
    <field name="velocity_y">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
    <field name="robot_name">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
    <field name="velocity_theta">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
    <field name="velocity_x">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
    <field name="stamp">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
  </fieldConfiguration>
  <geometryOptions removeDuplicateNodes="0" geometryPrecision="0"/>
  <aliases>
    <alias name="" field="velocity_y" index="0"/>
    <alias name="" field="robot_name" index="1"/>
    <alias name="" field="velocity_theta" index="2"/>
    <alias name="" field="velocity_x" index="3"/>
    <alias name="" field="stamp" index="4"/>
  </aliases>
  <excludeAttributesWMS/>
  <excludeAttributesWFS/>
  <defaults>
    <default expression="" field="velocity_y" applyOnUpdate="0"/>
    <default expression="" field="robot_name" applyOnUpdate="0"/>
    <default expression="" field="velocity_theta" applyOnUpdate="0"/>
    <default expression="" field="velocity_x" applyOnUpdate="0"/>
    <default expression="" field="stamp" applyOnUpdate="0"/>
  </defaults>
  <constraints>
    <constraint notnull_strength="0" unique_strength="0" constraints="0" field="velocity_y" exp_strength="0"/>
    <constraint notnull_strength="0" unique_strength="0" constraints="0" field="robot_name" exp_strength="0"/>
    <constraint notnull_strength="0" unique_strength="0" constraints="0" field="velocity_theta" exp_strength="0"/>
    <constraint notnull_strength="0" unique_strength="0" constraints="0" field="velocity_x" exp_strength="0"/>
    <constraint notnull_strength="0" unique_strength="0" constraints="0" field="stamp" exp_strength="0"/>
  </constraints>
  <constraintExpressions>
    <constraint desc="" exp="" field="velocity_y"/>
    <constraint desc="" exp="" field="robot_name"/>
    <constraint desc="" exp="" field="velocity_theta"/>
    <constraint desc="" exp="" field="velocity_x"/>
    <constraint desc="" exp="" field="stamp"/>
  </constraintExpressions>
  <attributeactions>
    <defaultAction value="{00000000-0000-0000-0000-000000000000}" key="Canvas"/>
  </attributeactions>
  <attributetableconfig sortExpression="" actionWidgetStyle="dropDown" sortOrder="0">
    <columns>
      <column name="velocity_y" width="-1" hidden="0" type="field"/>
      <column name="robot_name" width="-1" hidden="0" type="field"/>
      <column name="velocity_theta" width="-1" hidden="0" type="field"/>
      <column name="velocity_x" width="-1" hidden="0" type="field"/>
      <column name="stamp" width="-1" hidden="0" type="field"/>
      <column width="-1" hidden="1" type="actions"/>
    </columns>
  </attributetableconfig>
  <editform tolerant="1"></editform>
  <editforminit/>
  <editforminitcodesource>0</editforminitcodesource>
  <editforminitfilepath></editforminitfilepath>
  <editforminitcode><![CDATA[# -*- coding: utf-8 -*-
"""
QGIS forms can have a Python function that is called when the form is
opened.

Use this function to add extra logic to your forms.

Enter the name of the function in the "Python Init function"
field.
An example follows:
"""
from qgis.PyQt.QtWidgets import QWidget

def my_form_open(dialog, layer, feature):
	geom = feature.geometry()
	control = dialog.findChild(QWidget, "MyLineEdit")
]]></editforminitcode>
  <featformsuppress>0</featformsuppress>
  <editorlayout>generatedlayout</editorlayout>
  <editable>
    <field name="robot_name" editable="1"/>
    <field name="stamp" editable="1"/>
    <field name="velocity_theta" editable="1"/>
    <field name="velocity_x" editable="1"/>
    <field name="velocity_y" editable="1"/>
  </editable>
  <labelOnTop>
    <field name="robot_name" labelOnTop="0"/>
    <field name="stamp" labelOnTop="0"/>
    <field name="velocity_theta" labelOnTop="0"/>
    <field name="velocity_x" labelOnTop="0"/>
    <field name="velocity_y" labelOnTop="0"/>
  </labelOnTop>
  <widgets/>
  <conditionalstyles>
    <rowstyles/>
    <fieldstyles/>
  </conditionalstyles>
  <expressionfields/>
  <previewExpression>robot_name</previewExpression>
  <mapTip></mapTip>
  <layerGeometryType>0</layerGeometryType>
</qgis>
