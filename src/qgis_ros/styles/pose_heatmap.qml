<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis.dtd' 'SYSTEM'>
<qgis hasScaleBasedVisibilityFlag="0" simplifyDrawingTol="1" minScale="1e+08" simplifyAlgorithm="0" simplifyDrawingHints="0" readOnly="0" version="3.3.0-Master" simplifyMaxScale="1" simplifyLocal="1" labelsEnabled="0" maxScale="0">
  <renderer-v2 forceraster="0" radius="7" radius_unit="0" max_value="20" type="heatmapRenderer" radius_map_unit_scale="3x:0,0,0,0,0,0" weight_expression="" quality="3" enableorderby="0">
    <colorramp type="gradient" name="[source]">
      <prop k="color1" v="43,131,186,0"/>
      <prop k="color2" v="215,25,28,255"/>
      <prop k="discrete" v="0"/>
      <prop k="rampType" v="gradient"/>
      <prop k="stops" v="0.25;171,221,164,255:0.5;255,255,191,255:0.75;253,174,97,255"/>
    </colorramp>
  </renderer-v2>
  <customproperties>
    <property key="dualview/previewExpressions">
      <value>stamp</value>
    </property>
    <property value="0" key="embeddedWidgets/count"/>
    <property key="variableNames"/>
    <property key="variableValues"/>
  </customproperties>
  <blendMode>0</blendMode>
  <featureBlendMode>0</featureBlendMode>
  <layerOpacity>1</layerOpacity>
  <SingleCategoryDiagramRenderer attributeLegend="1" diagramType="Histogram">
    <DiagramCategory penWidth="0" scaleBasedVisibility="0" maxScaleDenominator="1e+08" minimumSize="0" backgroundAlpha="255" sizeType="MM" opacity="1" backgroundColor="#ffffff" lineSizeScale="3x:0,0,0,0,0,0" width="15" penAlpha="255" barWidth="5" labelPlacementMethod="XHeight" diagramOrientation="Up" penColor="#000000" scaleDependency="Area" lineSizeType="MM" minScaleDenominator="0" sizeScale="3x:0,0,0,0,0,0" rotationOffset="270" height="15" enabled="0">
      <fontProperties style="" description="Ubuntu,11,-1,5,50,0,0,0,0,0"/>
    </DiagramCategory>
  </SingleCategoryDiagramRenderer>
  <DiagramLayerSettings obstacle="0" showAll="1" placement="0" linePlacementFlags="18" priority="0" zIndex="0" dist="0">
    <properties>
      <Option type="Map">
        <Option value="" type="QString" name="name"/>
        <Option name="properties"/>
        <Option value="collection" type="QString" name="type"/>
      </Option>
    </properties>
  </DiagramLayerSettings>
  <fieldConfiguration>
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
    <field name="velocity_theta">
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
    <field name="velocity_y">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
  </fieldConfiguration>
  <geometryOptions removeDuplicateNodes="0" geometryPrecision="0"/>
  <aliases>
    <alias index="0" field="velocity_x" name=""/>
    <alias index="1" field="stamp" name=""/>
    <alias index="2" field="velocity_theta" name=""/>
    <alias index="3" field="robot_name" name=""/>
    <alias index="4" field="velocity_y" name=""/>
  </aliases>
  <excludeAttributesWMS/>
  <excludeAttributesWFS/>
  <defaults>
    <default applyOnUpdate="0" expression="" field="velocity_x"/>
    <default applyOnUpdate="0" expression="" field="stamp"/>
    <default applyOnUpdate="0" expression="" field="velocity_theta"/>
    <default applyOnUpdate="0" expression="" field="robot_name"/>
    <default applyOnUpdate="0" expression="" field="velocity_y"/>
  </defaults>
  <constraints>
    <constraint constraints="0" unique_strength="0" exp_strength="0" field="velocity_x" notnull_strength="0"/>
    <constraint constraints="0" unique_strength="0" exp_strength="0" field="stamp" notnull_strength="0"/>
    <constraint constraints="0" unique_strength="0" exp_strength="0" field="velocity_theta" notnull_strength="0"/>
    <constraint constraints="0" unique_strength="0" exp_strength="0" field="robot_name" notnull_strength="0"/>
    <constraint constraints="0" unique_strength="0" exp_strength="0" field="velocity_y" notnull_strength="0"/>
  </constraints>
  <constraintExpressions>
    <constraint exp="" desc="" field="velocity_x"/>
    <constraint exp="" desc="" field="stamp"/>
    <constraint exp="" desc="" field="velocity_theta"/>
    <constraint exp="" desc="" field="robot_name"/>
    <constraint exp="" desc="" field="velocity_y"/>
  </constraintExpressions>
  <attributeactions>
    <defaultAction value="{00000000-0000-0000-0000-000000000000}" key="Canvas"/>
  </attributeactions>
  <attributetableconfig sortExpression="" actionWidgetStyle="dropDown" sortOrder="0">
    <columns>
      <column hidden="0" width="-1" type="field" name="stamp"/>
      <column hidden="1" width="-1" type="actions"/>
      <column hidden="0" width="-1" type="field" name="velocity_x"/>
      <column hidden="0" width="252" type="field" name="velocity_theta"/>
      <column hidden="0" width="-1" type="field" name="robot_name"/>
      <column hidden="0" width="253" type="field" name="velocity_y"/>
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
    <field editable="1" name="robot_name"/>
    <field editable="1" name="stamp"/>
    <field editable="1" name="velocity_theta"/>
    <field editable="1" name="velocity_x"/>
    <field editable="1" name="velocity_y"/>
    <field editable="1" name="yaw"/>
  </editable>
  <labelOnTop>
    <field labelOnTop="0" name="robot_name"/>
    <field labelOnTop="0" name="stamp"/>
    <field labelOnTop="0" name="velocity_theta"/>
    <field labelOnTop="0" name="velocity_x"/>
    <field labelOnTop="0" name="velocity_y"/>
    <field labelOnTop="0" name="yaw"/>
  </labelOnTop>
  <widgets/>
  <conditionalstyles>
    <rowstyles/>
    <fieldstyles/>
  </conditionalstyles>
  <expressionfields/>
  <previewExpression>stamp</previewExpression>
  <mapTip></mapTip>
  <layerGeometryType>0</layerGeometryType>
</qgis>
